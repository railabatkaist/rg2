// /*
// This folder will EVENTUALLY contain all the Evaluation logic of the WalkerEnv.
// Evaluator-logic is abstracted in Environment, so once they are implemented,
// Environemnt has no idea how their internal works.

// - log(), they should check the world, robot state and
//     track some statistics. This includes stuff like torque usage, foot clearance, is_slipping, contact counts etc.
//     This will all eventually effect REWARD and GROUND_TRUTH, but not observation.
//     these function will only PRODUCE the tracked statistics.

// - evaluate(), they should return OBSERVATION and REWARD for RIGHT NOW.
//     This will calculate reward values based on its tracked logging statistics and values.
//     Each component of reward and observation should be calculated with internal .handleXXX() semantics.
//     Each .handleXXX() should only CONSUME the tracked statistics, and return the value.

// - reset(), they should reset the tracked statistics.

// All of these methods will not EVER change the world or robot state.
// This will be completely independent of the *how* environment is implemented or augmented.
// */

// #pragma once

// #include "../include/WalkerEnv.hpp"
// #include <deque>

// class WalkerEnvEvaluator
// {
// public:
//     WalkerEnvEvaluator(raisim::ArticulatedSystem &robot, raisim::World &world, std::vector<double> RewardCoeff)
//         : robot_(robot), world_(world), RewardCoeff_(RewardCoeff)
//     {
//         int jointNum = robot_.getDOF() - 6;
//         joint_torque_stats_.resize(jointNum);
//         joint_pos_stats_.resize(jointNum);
//         joint_speed_stats_.resize(jointNum);
//         joint_acc_stats_.resize(jointNum);
//         action_smoothness_stats_.resize(jointNum);
//         action_smoothness2_stats_.resize(jointNum);
//     }

//     void actionLogger(EigenVec action)
//     {
//         // If action history is full, remove the oldest action
//         if (action_history_.size() == action_history_capacity_)
//             action_history_.pop_front();

//         // Add new action to the history
//         action_history_.push_back(action);
//     }

//     void log()
//     {
//         // Update the log with the latest action
//         actionLogger(robot_.getGeneralizedForce().tail(robot_.getDOF() - 6));
//     }

//     std::tuple<EigenVec, double> evaluate()
//     {
//         double reward = 0.0;

//         // Calculate each component of reward
//         reward += rewardLinVelHandler();
//         reward += rewardAngVelHandler();
//         reward += rewardAirtimeHandler();
//         reward += rewardJointHandler();
//         reward += rewardActionSmoothnessHandler();
//         reward += rewardActionSmoothness2Handler();

//         EigenVec observation = robot_.getGeneralizedCoordinate().e();

//         return {observation, reward};
//     }

//     double rewardLinVelHandler()
//     {
//         return robot_.getBaseLinVel().norm() * RewardCoeff_[0];
//     }

//     double rewardAngVelHandler()
//     {
//         return robot_.getBaseAngVel().norm() * RewardCoeff_[1];
//     }

//     double rewardAirtimeHandler()
//     {
//         return (robot_.getContacts().empty() ? 1.0 : 0.0) * RewardCoeff_[2]; // assume airtime means no contacts
//     }

//     double rewardJointHandler()
//     {
//         double reward = 0.0;

//         // Retrieve generalized dynamics properties only once
//         const auto &gen_force = robot_.getGeneralizedForce().e();
//         const auto &gen_coord = robot_.getGeneralizedCoordinate().e();
//         const auto &gen_vel = robot_.getGeneralizedVelocity().e();

//         for (int i = 0; i < joint_torque_stats_.size(); ++i)
//         {
//             joint_torque_stats_[i] = std::abs(gen_force(i + 6)); // the first 6 DOFs are for the base
//             joint_pos_stats_[i] = std::abs(gen_coord(i + 6));
//             joint_speed_stats_[i] = std::abs(gen_vel(i + 6));

//             // Add joint-related rewards to total reward
//             reward += joint_torque_stats_[i] * RewardCoeff_[3];
//             reward += joint_pos_stats_[i] * RewardCoeff_[4];
//             reward += joint_speed_stats_[i] * RewardCoeff_[5];
//         }

//         return reward;
//     }

//     double rewardActionSmoothnessHandler()
//     {
//         double reward = 0.0;
//         for (int i = 0; i < action_smoothness_stats_.size(); ++i)
//         {
//             action_smoothness_stats_[i] = 0.0;

//             for (int j = 1; j < action_history_.size(); ++j)
//             {
//                 action_smoothness_stats_[i] += std::abs(action_history_[j][i] - action_history_[j - 1][i]);
//             }

//             reward += action_smoothness_stats_[i] * RewardCoeff_[8];
//         }
//         return reward;
//     }

//     double rewardActionSmoothness2Handler()
//     {
//         double reward = 0.0;
//         for (int i = 0; i < action_smoothness2_stats_.size(); ++i)
//         {
//             action_smoothness2_stats_[i] = 0.0;

//             for (int j = 1; j < action_history_.size(); ++j)
//             {
//                 action_smoothness2_stats_[i] += std::abs(action_history_[j][i] - 2 * action_history_[j - 1][i] + action_history_[j - 2][i]);
//             }

//             reward += action_smoothness2_stats_[i] * RewardCoeff_[9];
//         }
//         return reward;
//     }

//     void reset()
//     {
//         joint_torque_stats_.setZero();
//         joint_pos_stats_.setZero();
//         joint_speed_stats_.setZero();
//         joint_acc_stats_.setZero();
//         action_smoothness_stats_.setZero();
//         action_smoothness2_stats_.setZero();
//         action_history_.clear();
//     }

// private:
//     raisim::ArticulatedSystem &robot_;
//     raisim::World &world_;
//     std::vector<double> RewardCoeff_;
//     EigenVec joint_torque_stats_;
//     EigenVec joint_pos_stats_;
//     EigenVec joint_speed_stats_;
//     EigenVec joint_acc_stats_;
//     EigenVec action_smoothness_stats_;
//     EigenVec action_smoothness2_stats_;
//     std::deque<EigenVec> action_history_;
//     static constexpr int action_history_capacity_ = 3; // Change as per requirement
// };
