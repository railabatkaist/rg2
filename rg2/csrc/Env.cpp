//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//
#ifndef SRC_RAISIMGYMENV_HPP
#define SRC_RAISIMGYMENV_HPP

#pragma once

#include <stdlib.h>
#include <set>

#include <vector>
#include <memory>
#include <unordered_map>
#include "Common.hpp"
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "Yaml.hpp"
#include "Reward.hpp"
// #include "Env.hpp"

namespace raisim
{
  class UnitEnv
  {
  public:
    explicit UnitEnv(const std::string &resourceDir, const std::string &cfgString, bool visualizable) : resourceDir_(std::move(resourceDir)), visualizable_(visualizable), normDist_(0, 1)
    {
      Yaml::Parse(cfg_, cfgString);

      /// create world
      world_ = std::make_unique<raisim::World>();

      /// add objects
      robot_ = world_->addArticulatedSystem(resourceDir_);
      robot_->setName("robot");
      robot_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      world_->addGround();

      /// get robot data
      gcDim_ = robot_->getGeneralizedCoordinateDim();
      gvDim_ = robot_->getDOF();
      nJoints_ = gvDim_ - 6;

      /// initialize containers
      gc_.setZero(gcDim_);
      gcInit_.setZero(gcDim_);
      gv_.setZero(gvDim_);
      gvInit_.setZero(gvDim_);
      pTarget_.setZero(gcDim_);
      vTarget_.setZero(gvDim_);
      pTargetTail.setZero(nJoints_);

      /// this is nominal configuration of robot
      gcInit_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

      /// set pd gains
      Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
      jointPgain.setZero();
      jointPgain.tail(nJoints_).setConstant(50.0);
      jointDgain.setZero();
      jointDgain.tail(nJoints_).setConstant(0.2);
      robot_->setPdGains(jointPgain, jointDgain);
      robot_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

      /// MUST BE DONE FOR ALL UnitEnvS
      obDim_ = 1 + 3 + 3 + 3 + nJoints_ * 2;
      actionDim_ = nJoints_;
      actionMean_.setZero(actionDim_);
      actionStd_.setZero(actionDim_);
      obDouble_.setZero(obDim_);

      /// action scaling
      actionMean_ = gcInit_.tail(nJoints_);
      double action_std;
      READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
      actionStd_.setConstant(action_std);

      /// Reward coefficients
      rewards_.initializeFromConfigurationFile(cfg_["reward"]);

      /// indices of links that should not make contact with ground
      footIndices_.insert(robot_->getBodyIdx("LF_SHANK"));
      footIndices_.insert(robot_->getBodyIdx("RF_SHANK"));
      footIndices_.insert(robot_->getBodyIdx("LH_SHANK"));
      footIndices_.insert(robot_->getBodyIdx("RH_SHANK"));

      /// visualize if it is the first environment
      if (visualizable_)
      {
        server_ = std::make_unique<raisim::RaisimServer>(world_.get());
        server_->launchServer();
        server_->focusOn(robot_);
      }
    }
    ~UnitEnv()
    {
      if (server_)
        server_->killServer();
    };

    void setInitConstants(Eigen::VectorXd gcInit, Eigen::VectorXd gvInit, Eigen::VectorXd actionMean, Eigen::VectorXd actionStd, Eigen::VectorXd pGain, Eigen::VectorXd dGain)
    {
      assert(gcInit.size() == gcDim_);
      assert(gvInit.size() == gvDim_);
      assert(actionMean.size() == actionDim_);
      assert(actionStd.size() == actionDim_);
      assert(pGain.size() == gvDim_);
      assert(dGain.size() == gvDim_);

      // set gc and gv init value
      gcInit_ = gcInit;
      gvInit_ = gvInit;

      // set action mean and std
      actionMean_ = actionMean;
      actionStd_ = actionStd;

      // set PD gains
      robot_->setPdGains(pGain, dGain);
    }

    void init()
    {
      robot_->setState(gcInit_, gvInit_);
      updateObservation();
    }

    void reset()
    {
      robot_->setState(gcInit_, gvInit_);
      updateObservation();
    }

    float step(const Eigen::Ref<EigenVec> &action)
    {
      /// action scaling
      pTargetTail = action.cast<double>();
      pTargetTail = pTargetTail.cwiseProduct(actionStd_);
      pTargetTail += actionMean_;
      pTarget_.tail(nJoints_) = pTargetTail;

      robot_->setPdTarget(pTarget_, vTarget_);

      for (int i = 0; i < int(control_dt_ / simulation_dt_ + 1e-10); i++)
      {
        if (server_)
          server_->lockVisualizationServerMutex();
        world_->integrate();
        if (server_)
          server_->unlockVisualizationServerMutex();
      }

      updateObservation();

      rewards_.record("torque", robot_->getGeneralizedForce().squaredNorm());
      rewards_.record("forwardVel", std::min(4.0, bodyLinearVel_[0]));

      return rewards_.sum();
    }

    void updateObservation()
    {
      robot_->getState(gc_, gv_);
      raisim::Vec<4> quat;
      raisim::Mat<3, 3> rot;
      quat[0] = gc_[3];
      quat[1] = gc_[4];
      quat[2] = gc_[5];
      quat[3] = gc_[6];
      raisim::quatToRotMat(quat, rot);
      bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
      bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

      obDouble_ << gc_[2],                 /// body height
          rot.e().row(2).transpose(),      /// body orientation
          gc_.tail(nJoints_),              /// joint angles
          bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
          gv_.tail(nJoints_);              /// joint velocity
    }

    void observe(Eigen::Ref<EigenVec> ob)
    {
      /// convert it to float
      ob = obDouble_.cast<float>();
    }

    bool isTerminalState(float &terminalReward)
    {
      terminalReward = float(terminalRewardCoeff_);

      /// if the contact body is not feet
      for (auto &contact : robot_->getContacts())
        if (footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
          return true;

      terminalReward = 0.f;
      return false;
    }

    void curriculumUpdate(){};

    void setSimulationTimeStep(double dt)
    {
      simulation_dt_ = dt;
      world_->setTimeStep(dt);
    }

    void close(){};
    void setSeed(int seed){};

    void setControlTimeStep(double dt) { control_dt_ = dt; }
    int getObDim() { return obDim_; }
    int getActionDim() { return actionDim_; }
    double getControlTimeStep() { return control_dt_; }
    double getSimulationTimeStep() { return simulation_dt_; }
    raisim::World *getWorld() { return world_.get(); }
    void turnOffVisualization() { server_->hibernate(); }
    void turnOnVisualization() { server_->wakeup(); }
    void startRecordingVideo(const std::string &videoName) { server_->startRecordingVideo(videoName); }
    void stopRecordingVideo() { server_->stopRecordingVideo(); }
    raisim::Reward &getRewards() { return rewards_; }

  private:
    int gcDim_, gvDim_, nJoints_;
    bool visualizable_ = false;
    raisim::ArticulatedSystem *robot_;
    Eigen::VectorXd gcInit_, gvInit_, gc_, gv_, pTarget_, pTargetTail, vTarget_;
    double terminalRewardCoeff_ = -10.;
    Eigen::VectorXd actionMean_, actionStd_, obDouble_;
    Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
    std::set<size_t> footIndices_;

    /// these variables are not in use. They are placed to show you how to create a random number sampler.
    std::normal_distribution<double> normDist_;
    thread_local static std::mt19937 gen_;
    std::unique_ptr<raisim::World> world_;
    double simulation_dt_ = 0.001;
    double control_dt_ = 0.01;
    std::string resourceDir_;
    Yaml::Node cfg_;
    int obDim_ = 0, actionDim_ = 0;
    std::unique_ptr<raisim::RaisimServer> server_;
    raisim::Reward rewards_;
  };
}

#endif // SRC_RAISIMGYMENV_HPP