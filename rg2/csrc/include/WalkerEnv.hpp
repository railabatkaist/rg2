#pragma once

#include <stdlib.h>
#include <set>

#include <vector>
#include <memory>
#include <unordered_map>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

using Dtype = float;
using EigenRowMajorMat = Eigen::Matrix<Dtype, -1, -1, Eigen::RowMajor>;
using EigenVec = Eigen::Matrix<Dtype, -1, 1>;
using EigenBoolVec = Eigen::Matrix<bool, -1, 1>;

// Sampler

Eigen::MatrixXd sampleGaussian(const Eigen::Index rows, const Eigen::Index cols, const double mean, const double stddev, std::mt19937 &generator);

Eigen::MatrixXd sampleUniform(const Eigen::Index rows, const Eigen::Index cols, const double min_val, const double max_val, std::mt19937 &generator);

enum class EnvParams
{
  NOISE_ACTION_DELAY = 0,
  NOISE_OBSERVATION = 1,
  NOISE_MOTOR_FRICTION = 2,     // [-1, 1]
  NOISE_PD_CONTROLLER_GAIN = 3, // [-1, 1]
  NOISE_GROND_FRICTION = 4,     // [-1, 1]
  COMMAND_LEVEL = 5,            // [-1, 1]
  SIZE = 6,
};

struct WalkerEnvConfig
{
  std::string resourceDir;
  Eigen::VectorXd gcInit;
  Eigen::VectorXd gvInit;
  Eigen::VectorXd actionMean;
  Eigen::VectorXd actionStd;
  float pGain;
  float dGain;
  std::vector<double> envParams;

  WalkerEnvConfig(py::dict config)
  {
    // Check that each key exists before trying to access it.
    assert(config.contains("resourceDir"));
    assert(config.contains("gcInit"));
    assert(config.contains("gvInit"));
    assert(config.contains("actionMean"));
    assert(config.contains("actionStd"));
    assert(config.contains("pGain"));
    assert(config.contains("dGain"));
    assert(config.contains("envParams"));

    resourceDir = config["resourceDir"].cast<std::string>();
    gcInit = config["gcInit"].cast<Eigen::VectorXd>();
    gvInit = config["gvInit"].cast<Eigen::VectorXd>();
    actionMean = config["actionMean"].cast<Eigen::VectorXd>();
    actionStd = config["actionStd"].cast<Eigen::VectorXd>();
    pGain = config["pGain"].cast<float>();
    dGain = config["dGain"].cast<float>();
    envParams = config["envParams"].cast<std::vector<double>>();

    // // print all
    // std::cout << "resourceDir: " << resourceDir << std::endl;
    // std::cout << "gcInit: " << gcInit.transpose() << std::endl;
    // std::cout << "gvInit: " << gvInit.transpose() << std::endl;
    // std::cout << "actionMean: " << actionMean.transpose() << std::endl;
    // std::cout << "actionStd: " << actionStd.transpose() << std::endl;
    // std::cout << "pGain: " << pGain << std::endl;
    // std::cout << "dGain: " << dGain << std::endl;
    // std::cout << "envParams: " << std::endl;
    // for (auto &i : envParams)
    //   std::cout << i << " ";

    assert(envParams.size() == static_cast<int>(EnvParams::SIZE));
  }
};

class WalkerEnv
{
public:
  WalkerEnv(WalkerEnvConfig config, bool visualizable)
      : config_(config),
        visualizable_(visualizable),
        resourceDir_(config.resourceDir),
        gcInit_(config.gcInit),
        gvInit_(config.gvInit),
        actionMean_(config.actionMean),
        actionStd_(config.actionStd),
        envParams(config.envParams),
        robot_(nullptr)
  {

    createWorldAndRobot();

    if (visualizable_)
    {
      VIZDEBUG = true;
      std::cout << "at init, got";
      std::cout << "gcInit: " << gcInit_.transpose() << std::endl;
      std::cout << "gvInit: " << gvInit_.transpose() << std::endl;
      std::cout << "actionMean: " << actionMean_.transpose() << std::endl;
      std::cout << "actionStd: " << actionStd_.transpose() << std::endl;
    }

    setInitConstants();
    setPdGains(config.pGain, config.dGain);
    reset();

    if (visualizable_)
      initializeVisualization();
  }
  ~WalkerEnv();

  void setInitConstants();
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
  thread_local static std::mt19937 gen_;

private:
  inline double envval(EnvParams curtype) { return envParams[static_cast<int>(curtype)]; }
  void setPdGains(float pGain, float dGain);
  void updateObservation();
  void createWorldAndRobot();
  void initializeVisualization();
  void simulateJointFriction();

  std::tuple<Eigen::VectorXd, Eigen::VectorXd> handleAction(const Eigen::Ref<EigenVec> &action);

  void accumulateSim(raisim::World *world, raisim::HeightMap *heightMap);

  void afterStep(raisim::World *world, raisim::HeightMap *heightMap);

  int gcDim_{0}, gvDim_{0}, nJoints_{0};
  bool visualizable_{false};

  raisim::ArticulatedSystem *robot_{nullptr};
  WalkerEnvConfig config_;
  Eigen::VectorXd gcInit_,
      gvInit_, gc_, gv_;
  double terminalRewardCoeff_{-10.0};
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;

  std::normal_distribution<double> normDist_;

  std::unique_ptr<raisim::World> world_;
  double simulation_dt_{0.001};
  double control_dt_{0.01};

  std::vector<double> envParams;

  std::string resourceDir_;

  int obDim_{0}, actionDim_{0};
  bool VIZDEBUG{false};

  std::unique_ptr<raisim::RaisimServer> server_;
};
