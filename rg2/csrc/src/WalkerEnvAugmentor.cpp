/*

This file will eventually contain all Environment Augmentation Logic.
Basically, once we are given set of envparam, it will uniquely define how environment will behave.
(i.e., envparam will be sufficient statistics of environment)
This implies that how environment gets augmented completely depends on envparam.
Unlike the name suggests, Augmentor handles both *ground truth* and *noise*.
For example, friction is augmented, but it is not noise.
On the other hand, action and observation are augmented with noise, and is due to practicality of the robotic system.

EnvAugmentor will have following methods:

.constructor()
    This will set envparam of the environment. This will be called at the beginning of the episode.
    This *must* call augGlobal.

.augGlobal()
    This will augment global variables of the environment.This will modify stuff like

    * ground friction
    * PD controller gains
    * Map of the environment

.augLocal()
    This will augment local variables (per-timestep) of the environment.
    This include stuff like joint friction, time-step etc.

.augActionPipe()
    Will augment action pipeline. This will make action noisy.
    This simulates the error of the mechanical system.

.augObservationPipe()
    Will augment observation pipeline. This will make observation noisy.
    This simulates the error of the sensor system.

*/

#include "../include/WalkerEnv.hpp"

void WalkerEnv::simulateJointFriction()
{
    // This is done BEFORE integrate, and AFTER jointFriction Settings.

    Eigen::VectorXd frictionTorques_;
    Eigen::VectorXd gf_ = robot_->getGeneralizedForce().e();
    frictionTorques_.setZero(gvDim_);
    // Using constant for friction coefficient
    const double frictionCoeff = 0.15;

    // Generate random values using uniform distribution
    Eigen::MatrixXd randomValues = sampleUniform(nJoints_, 1, 0.0, 1.0, gen_) + Eigen::MatrixXd::Constant(nJoints_, 1, 1.0);

    // Calculate joint frictions using broadcasting
    Eigen::VectorXd jointFrictions = frictionCoeff * envval(EnvParams::NOISE_MOTOR_FRICTION) * randomValues.col(0);

    // Calculate absolute generalized force values and pre-calculate signs
    Eigen::VectorXd absGFValues = gf_.segment(6, nJoints_).cwiseAbs();
    Eigen::VectorXd signs = -gf_.segment(6, nJoints_).array().sign();

    // Calculate friction torques using broadcasting and ternary operation
    frictionTorques_.segment(6, nJoints_) =
        (absGFValues.array() > jointFrictions.array()).select(jointFrictions.cwiseProduct(signs), -gf_.segment(6, nJoints_));

    robot_->setGeneralizedForce(frictionTorques_);
}
