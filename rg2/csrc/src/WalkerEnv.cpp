#include "../include/WalkerEnv.hpp"
thread_local std::mt19937 WalkerEnv::gen_;

// Function to sample from a Gaussian distribution
Eigen::MatrixXd sampleGaussian(const Eigen::Index rows, const Eigen::Index cols, const double mean, const double stddev, std::mt19937 &generator)
{
    std::normal_distribution<double> dist(mean, stddev);
    Eigen::MatrixXd result(rows, cols);

    for (Eigen::Index i = 0; i < rows; ++i)
    {
        for (Eigen::Index j = 0; j < cols; ++j)
        {
            result(i, j) = dist(generator);
        }
    }

    return result;
}

// Function to sample from a uniform distribution
Eigen::MatrixXd sampleUniform(const Eigen::Index rows, const Eigen::Index cols, const double min_val, const double max_val, std::mt19937 &generator)
{
    std::uniform_real_distribution<double> dist(min_val, max_val);
    Eigen::MatrixXd result(rows, cols);

    for (Eigen::Index i = 0; i < rows; ++i)
    {
        for (Eigen::Index j = 0; j < cols; ++j)
        {
            result(i, j) = dist(generator);
        }
    }

    return result;
}

WalkerEnv::~WalkerEnv()
{
    if (server_)
        server_->killServer();
}

void WalkerEnv::createWorldAndRobot()
{
    world_ = std::make_unique<raisim::World>();
    robot_ = world_->addArticulatedSystem(resourceDir_);
    robot_->setName("robot");
    robot_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();
    gcDim_ = robot_->getGeneralizedCoordinateDim();
    gvDim_ = robot_->getDOF();
    nJoints_ = gvDim_ - 6;
    gc_.setZero(gcDim_);
    gv_.setZero(gvDim_);
}

void WalkerEnv::setPdGains(float pGain, float dGain)
{
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero();
    jointPgain.tail(nJoints_).setConstant(pGain);
    jointDgain.setZero();
    jointDgain.tail(nJoints_).setConstant(dGain);

    // noise
    jointPgain.tail(nJoints_) += jointPgain.tail(nJoints_).cwiseProduct(Eigen::VectorXd::Random(nJoints_) * 0.05 * (envval(EnvParams::NOISE_PD_CONTROLLER_GAIN) + 1));
    jointDgain.tail(nJoints_) += jointDgain.tail(nJoints_).cwiseProduct(Eigen::VectorXd::Random(nJoints_) * 0.05 * (envval(EnvParams::NOISE_PD_CONTROLLER_GAIN) + 1));

    if (VIZDEBUG)
    {
        std::cout << "At SetPDGAIN jointPgain: " << jointPgain.transpose() << std::endl;
        std::cout << "At SetPDGAIN jointDgain: " << jointDgain.transpose() << std::endl;
    }

    robot_->setPdGains(jointPgain, jointDgain);
    robot_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
}
// Function to print the error message
void printAssertionError(const std::string &errorMsg)
{
    std::cout << "Assertion failed: " << errorMsg << std::endl;
}
void WalkerEnv::initializeVisualization()
{
    std::cout << "Starting visualization thread..." << std::endl;
    server_ = std::make_unique<raisim::RaisimServer>(world_.get());
    server_->launchServer();
    server_->focusOn(robot_);
}

void WalkerEnv::setInitConstants()
{
    // Calculate obDim_ and actionDim_ based on nJoints_
    obDim_ = 1 + 3 + 3 + 3 + nJoints_ * 2;
    actionDim_ = nJoints_;

    // Initialize obDouble_ with zeros
    obDouble_.setZero(obDim_);

    // Insert foot indices into footIndices_ set
    footIndices_.insert(robot_->getBodyIdx("LF_SHANK"));
    footIndices_.insert(robot_->getBodyIdx("RF_SHANK"));
    footIndices_.insert(robot_->getBodyIdx("LH_SHANK"));
    footIndices_.insert(robot_->getBodyIdx("RH_SHANK"));

    // Check sizes and print error messages if assertions fail
    if (gcInit_.size() != gcDim_)
    {
        printAssertionError("Invalid size for gcInit_ vector. Expected size: " + std::to_string(gcDim_) + ", Actual size: " + std::to_string(gcInit_.size()));
    }
    if (gvInit_.size() != gvDim_)
    {
        printAssertionError("Invalid size for gvInit_ vector. Expected size: " + std::to_string(gvDim_) + ", Actual size: " + std::to_string(gvInit_.size()));
    }
    if (actionMean_.size() != actionDim_)
    {
        printAssertionError("Invalid size for actionMean_ vector. Expected size: " + std::to_string(actionDim_) + ", Actual size: " + std::to_string(actionMean_.size()));
    }
    if (actionStd_.size() != actionDim_)
    {
        printAssertionError("Invalid size for actionStd_ vector. Expected size: " + std::to_string(actionDim_) + ", Actual size: " + std::to_string(actionStd_.size()));
    }
    // Optional: Print debug information if VIZDEBUG is enabled
    if (VIZDEBUG)
    {
        std::cout << "setInitConstants :gcInit_: " << gcInit_.transpose() << std::endl;
        std::cout << "setInitConstants :gvInit_: " << gvInit_.transpose() << std::endl;
        std::cout << "setInitConstants :actionMean_: " << actionMean_.transpose() << std::endl;
        std::cout << "setInitConstants :actionStd_: " << actionStd_.transpose() << std::endl;
    }
}

void WalkerEnv::reset()
{
    // at reset, all the randomly set variables should be reset.
    double c_f = 0.7 + envval(EnvParams::NOISE_GROND_FRICTION) * 0.3;

    world_->setDefaultMaterial(c_f, 0.0, 0.01);
    if (VIZDEBUG)
    {
        std::cout << "RESET: gcInit_: " << gcInit_.transpose() << std::endl;
        std::cout << "RESELT :gvInit_: " << gvInit_.transpose() << std::endl;
    }

    robot_->setState(gcInit_, gvInit_);

    if (VIZDEBUG)
    {
        std::cout << "RESET: gcInit_: " << gcInit_.transpose() << std::endl;
        std::cout << "RESELT :gvInit_: " << gvInit_.transpose() << std::endl;
    }
    updateObservation();
}

// Transform action into PD target, and save statistics.
std::tuple<Eigen::VectorXd, Eigen::VectorXd> WalkerEnv::handleAction(const Eigen::Ref<EigenVec> &action)
{
    Eigen::VectorXd pTargetTail = action.cast<double>();
    pTargetTail = pTargetTail.cwiseProduct(actionStd_);
    pTargetTail += actionMean_;
    Eigen::VectorXd pTarget, dTarget;
    pTarget.setZero(gcDim_);
    dTarget.setZero(gvDim_);
    pTarget.tail(nJoints_) = pTargetTail;
    return std::make_tuple(pTarget, dTarget);
}

float WalkerEnv::step(const Eigen::Ref<EigenVec> &action)
{
    // It takes small time for action to be applied to the robot.

    int nSubSteps = int(control_dt_ / simulation_dt_ + 1e-10);
    int happening_step = 0;
    // power distribution of happening step
    double difficulty = (envval(EnvParams::NOISE_ACTION_DELAY) + 1.0) * 0.2 * (double)nSubSteps;
    int maxHappeningStep = nSubSteps - 2;

    std::poisson_distribution<int> dist(difficulty);
    happening_step = std::min(dist(gen_), maxHappeningStep);

    for (int i = 0; i < nSubSteps; i++)
    {
        if (server_)
            server_->lockVisualizationServerMutex();

        simulateJointFriction();

        world_->integrate();

        if (i == happening_step)
        {
            auto [pTarget, dTarget] = handleAction(action);
            if (VIZDEBUG)
            {
                std::cout << "At action: pTarget: " << pTarget.transpose() << std::endl;
                std::cout << "At action: dTarget: " << dTarget.transpose() << std::endl;
            }
            robot_->setPdTarget(pTarget, dTarget);
        }

        if (server_)
            server_->unlockVisualizationServerMutex();
    }
    updateObservation();

    float re = -4e-5 * robot_->getGeneralizedForce().squaredNorm() +
               0.3 * std::min(4.0, bodyLinearVel_[1]);

    return re;
}

void WalkerEnv::updateObservation()
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
    if (VIZDEBUG)
    {
        std::cout << "At Update Obs gc[2]: " << gc_[2] << std::endl;
        std::cout << "At Update Obs rot.e().row(2).transpose(): " << rot.e().row(2).transpose() << std::endl;
        std::cout << "At Update Obs gc_.tail(nJoints_): " << gc_.tail(nJoints_).transpose() << std::endl;
        std::cout << "At Update Obs bodyLinearVel_: " << bodyLinearVel_.transpose() << std::endl;
        std::cout << "At Update Obs bodyAngularVel_: " << bodyAngularVel_.transpose() << std::endl;
        std::cout << "At Update Obs gv_.tail(nJoints_): " << gv_.tail(nJoints_).transpose() << std::endl;
    }
    // noise
    obDouble_ += obDouble_.cwiseProduct(Eigen::VectorXd::Random(obDouble_.size()) * 0.05 * (envval(EnvParams::NOISE_OBSERVATION) + 1));

    assert(obDouble_.size() == obDim_);
}

void WalkerEnv::observe(Eigen::Ref<EigenVec> ob)
{
    ob = obDouble_.cast<float>();
}

bool WalkerEnv::isTerminalState(float &terminalReward)
{
    terminalReward = float(terminalRewardCoeff_);

    // /// if the contact body is not feet
    // for (auto &contact : robot_->getContacts())
    //     if (footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
    //         return true;

    const auto baseOri = robot_->getBaseOrientation();
    if (baseOri[8] < 0.6)
        return true;

    terminalReward = 0.f;
    return false;
}

void WalkerEnv::setSimulationTimeStep(double dt)
{
    simulation_dt_ = dt;
    world_->setTimeStep(dt);
}

void WalkerEnv::close() {}

void WalkerEnv::setSeed(int seed) {}

void WalkerEnv::setControlTimeStep(double dt)
{
    control_dt_ = dt;
}

int WalkerEnv::getObDim()
{
    return obDim_;
}

int WalkerEnv::getActionDim()
{
    return actionDim_;
}

double WalkerEnv::getControlTimeStep()
{
    return control_dt_;
}

double WalkerEnv::getSimulationTimeStep()
{
    return simulation_dt_;
}

raisim::World *WalkerEnv::getWorld()
{
    return world_.get();
}

void WalkerEnv::turnOffVisualization()
{
    server_->hibernate();
}

void WalkerEnv::turnOnVisualization()
{
    server_->wakeup();
}

void WalkerEnv::startRecordingVideo(const std::string &videoName)
{
    server_->startRecordingVideo(videoName);
}

void WalkerEnv::stopRecordingVideo()
{
    server_->stopRecordingVideo();
}
