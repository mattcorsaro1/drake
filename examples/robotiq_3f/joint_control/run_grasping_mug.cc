/// @file
///
/// This file set up an example about control the robotiq_3f hand based on
/// position. In the program, the hand grasps a mug and releases several times,
/// using a different grasp mode each time. The program presently only runs
/// on simulation, with the file robotiq_3f_single_object_simulation.cc which
/// creates the simulation environment for the hand and object. This program
/// reads from LCM about the state of the hands, and process command the
/// positions of the finger joints through LCM.
/// It also uses the velocity states of the fingers to decide whether the hand
/// has finished the current motion, either by reaching the target position or
/// get stuck by collisions.
///
/// originally copied from examples/allegro_hand @ f646302

#include <Eigen/Dense>
#include "lcm/lcm-cpp.hpp"

#include "drake/examples/robotiq_3f/robotiq_3f_common.h"
#include "drake/examples/robotiq_3f/robotiq_3f_lcm.h"
#include "drake/lcmt_robotiq_3f_command.hpp"
#include "drake/lcmt_robotiq_3f_status.hpp"

namespace drake {
namespace examples {
namespace robotiq_3f {
namespace {

const char* const kLcmStatusChannel = "ROBOTIQ3F_STATUS";
const char* const kLcmCommandChannel = "ROBOTIQ3F_COMMAND";

class PositionCommander {
 public:
  PositionCommander() {
    lcm_.subscribe(kLcmStatusChannel, &PositionCommander::HandleStatus,
                   this);
  }

  void Run() {
    robotiq_3f_command_.num_joints = kRobotiq3fNumJoints;
    robotiq_3f_command_.joint_position.resize(kRobotiq3fNumJoints, 0.);
    robotiq_3f_command_.num_torques = 0;
    robotiq_3f_command_.joint_torque.resize(0);

    flag_moving = true;
    Eigen::VectorXd target_joint_position(kRobotiq3fNumJoints);
    target_joint_position.setZero();

    std::cout << "Initial LCM State:" << std::endl;
    printLCMState();

    // Open hand in basic mode
    int grasp_mode_index = 0;
    target_joint_position = hand_state_.OpenJointPosition();
    MovetoPositionUntilStuck(target_joint_position);
    std::cout << "Hand is open." << std::endl;

    // Close hand in basic mode
    target_joint_position = hand_state_.GraspJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);
    std::cout << "Hand is closed." << std::endl;

    printLCMState();
    Eigen::VectorXd current_joint_position = getJointValuesOnceStatic();
    std::cout << "Current joint position:" << current_joint_position << std::endl;

    /*
    // Open hand in basic mode
    target_joint_position = hand_state_.OpenJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);

    // Open hand in wide mode
    grasp_mode_index = 1;
    target_joint_position = hand_state_.OpenJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);

    // Close hand in wide mode
    target_joint_position = hand_state_.GraspJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);
    std::cout << "Hand is closed. \n";
    while (0 == lcm_.handleTimeout(10)) {
    }

    // Open hand in wide mode
    target_joint_position = hand_state_.OpenJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);

    // Open hand in pinch mode
    grasp_mode_index = 2;
    target_joint_position = hand_state_.OpenJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);

    // Close hand in pinch mode
    target_joint_position = hand_state_.GraspJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);
    std::cout << "Hand is closed. \n";
    while (0 == lcm_.handleTimeout(10)) {
    }

    // Open hand in pinch mode
    target_joint_position = hand_state_.OpenJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);

    // Open hand in basic mode
    grasp_mode_index = 0;
    target_joint_position = hand_state_.OpenJointPosition(grasp_mode_index);
    MovetoPositionUntilStuck(target_joint_position);
    */
  }

 private:
  inline void PublishPositionCommand(
      const Eigen::VectorXd& target_joint_position) {
    Eigen::VectorXd::Map(&robotiq_3f_command_.joint_position[0],
                         kRobotiq3fNumJoints) = target_joint_position;
    lcm_.publish(kLcmCommandChannel, &robotiq_3f_command_);
  }

  inline void MovetoPositionUntilStuck(
      const Eigen::VectorXd& target_joint_position) {
    PublishPositionCommand(target_joint_position);
    // A time delay at the initial moving stage so that the noisy data from the
    // hand motion is filtered.
    for (int i = 0; i < 60; i++) {
      while (0 == lcm_.handleTimeout(10) || robotiq_3f_status_.utime == -1) {
      }
    }
    // wait until the fingers are stuck, or stop moving.
    while (flag_moving) {
      while (0 == lcm_.handleTimeout(10) || robotiq_3f_status_.utime == -1) {
      }
    }
  }

  // Returns last measured joint values
  Eigen::VectorXd getJointValuesOnceStatic() {
    return Eigen::Map<Eigen::VectorXd>(
        &(robotiq_3f_status_.joint_position_measured[0]), kRobotiq3fNumJoints);
  }

  void printVector(const std::vector<double>& vector_to_print) {
    std::cout << "(size " << vector_to_print.size() << "): ";
    for (std::vector<double>::const_iterator it = vector_to_print.begin();
      it != vector_to_print.end(); ++it) {
      std::cout << *it << " ";
    }
    std::cout << std::endl;
  }
  void printLCMState() {
    std::cout << "LCMState "  << robotiq_3f_status_.utime << std::endl;
    std::cout << "joint_position_measured ";
    printVector(robotiq_3f_status_.joint_position_measured);
    std::cout << "joint_position_commanded ";
    printVector(robotiq_3f_status_.joint_position_commanded);
    std::cout << "joint_torque_commanded ";
    printVector(robotiq_3f_status_.joint_torque_commanded);
    std::cout << "joint_velocity_estimated ";
    printVector(robotiq_3f_status_.joint_velocity_estimated);
  }

  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_robotiq_3f_status* status) {
    robotiq_3f_status_ = *status;
    hand_state_.Update(robotiq_3f_status_);
    flag_moving = !hand_state_.IsAllFingersStuck();
  }

  ::lcm::LCM lcm_;
  lcmt_robotiq_3f_status robotiq_3f_status_;
  lcmt_robotiq_3f_command robotiq_3f_command_;
  Robotiq3fHandMotionState hand_state_;

  bool flag_moving = true;
};

int do_main() {
  PositionCommander runner;
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::robotiq_3f::do_main(); }
