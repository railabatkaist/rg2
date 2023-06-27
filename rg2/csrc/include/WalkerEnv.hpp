#pragma once

#include <stdlib.h>
#include <set>

#include <vector>
#include <memory>
#include <unordered_map>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

using Dtype = float;
using EigenRowMajorMat = Eigen::Matrix<Dtype, -1, -1, Eigen::RowMajor>;
using EigenVec = Eigen::Matrix<Dtype, -1, 1>;
using EigenBoolVec = Eigen::Matrix<bool, -1, 1>;

class WalkerEnv
{
public:
  explicit WalkerEnv(const std::string &resourceDir, bool visualizable);
  ~WalkerEnv();

  void setInitConstants(Eigen::VectorXd gcInit, Eigen::VectorXd gvInit, Eigen::VectorXd actionMean, Eigen::VectorXd actionStd, Eigen::VectorXd pGain, Eigen::VectorXd dGain);
  void init();
  void reset();
  float step(const Eigen::Ref<EigenVec> &action);
  void observe(Eigen::Ref<EigenVec> ob);
  bool isTerminalState(float &terminalReward);
  void curriculumUpdate();

  void setSimulationTimeStep(double dt);
  void close();
  void setSeed(int seed);
  void setControlTimeStep(double dt);

  int getObDim();
  int getActionDim();
  double getControlTimeStep();
  double getSimulationTimeStep();

  raisim::World *getWorld();

  void turnOffVisualization();
  void turnOnVisualization();
  void startRecordingVideo(const std::string &videoName);
  void stopRecordingVideo();

private:
  void setPdGains();
  void updateObservation();
  void initializeContainers();
  void createWorldAndRobot();
  void initializeObservationSpace();
  void initializeVisualization();

  int gcDim_{0}, gvDim_{0}, nJoints_{0};
  bool visualizable_{false};

  raisim::ArticulatedSystem *robot_{nullptr};

  Eigen::VectorXd gcInit_, gvInit_, gc_, gv_, pTarget_, vTarget_;
  double terminalRewardCoeff_{-10.0};
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;

  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;

  std::unique_ptr<raisim::World> world_;
  double simulation_dt_{0.001};
  double control_dt_{0.01};

  std::string resourceDir_;

  int obDim_{0}, actionDim_{0};

  std::unique_ptr<raisim::RaisimServer> server_;
};
