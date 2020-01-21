// originally copied from examples/allegro_hand @ f646302

#include "drake/examples/robotiq_3f/robotiq_3f_common.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

const double Robotiq3fHandMotionState::velocity_thresh_ = 0.07;

using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;

void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                Eigen::VectorXd* Kd) {
  *Kp = Eigen::VectorXd::Ones(kRobotiq3fNumJoints) * 0.05;
  *Kd = Eigen::VectorXd::Constant(Kp->size(), 5e-3);
  (*Kp)[0] = 0.08;
  *Ki = Eigen::VectorXd::Zero(kRobotiq3fNumJoints);
}

std::vector<std::string> GetPreferredJointOrdering() {
  std::vector<std::string> joint_name_mapping;

  // finger_middle - index 0
  joint_name_mapping.push_back("finger_middle_joint_1");
  joint_name_mapping.push_back("finger_middle_joint_2");
  joint_name_mapping.push_back("finger_middle_joint_3");

  // finger_1 - index 1
  joint_name_mapping.push_back("finger_1_joint_1");
  joint_name_mapping.push_back("finger_1_joint_2");
  joint_name_mapping.push_back("finger_1_joint_3");

  // finger_2 - index 2
  joint_name_mapping.push_back("finger_2_joint_1");
  joint_name_mapping.push_back("finger_2_joint_2");
  joint_name_mapping.push_back("finger_2_joint_3");

  // palm - index 3
  joint_name_mapping.push_back("palm_finger_1_joint");
  joint_name_mapping.push_back("palm_finger_2_joint");

  return joint_name_mapping;
}

void GetControlPortMapping(
    const MultibodyPlant<double>& plant,
    MatrixX<double>* Sx, MatrixX<double>* Sy) {
  // Retrieve the list of finger joints in a user-defined ordering.
  const std::vector<std::string> joints_in_preferred_order =
      GetPreferredJointOrdering();

  // Make a list of the same joints but by JointIndex.
  std::vector<JointIndex> joint_index_mapping;
  for (const auto& joint_name : joints_in_preferred_order) {
    joint_index_mapping.push_back(plant.GetJointByName(joint_name).index());
  }

  *Sx = plant.MakeStateSelectorMatrix(joint_index_mapping);
  *Sy = plant.MakeActuatorSelectorMatrix(joint_index_mapping);
}

Robotiq3fHandMotionState::Robotiq3fHandMotionState()
    // TODO(mcorsaro): determine appropriate value for finger_num_. Currently
    // 3 real fingers plus scissoring mechanisms
    : finger_num_(4),
      is_joint_stuck_(robotiq_3f_num_joints_),
      is_finger_stuck_(finger_num_) {}

void Robotiq3fHandMotionState::Update(
    const lcmt_robotiq_3f_status& robotiq_3f_state_msg) {
  const lcmt_robotiq_3f_status status = robotiq_3f_state_msg;

  const double* ptr = &(status.joint_velocity_estimated[0]);
  const Eigen::ArrayXd joint_velocity =
      Eigen::Map<const Eigen::ArrayXd>(ptr, robotiq_3f_num_joints_);
  const Eigen::ArrayXd torque_command = Eigen::Map<const Eigen::ArrayXd>(
      &(status.joint_torque_commanded[0]), robotiq_3f_num_joints_);

  is_joint_stuck_ = joint_velocity.abs() < velocity_thresh_;

  // Detect whether the joint is moving in the opposite direction of the
  // command. If yes, it is most likely the joint is stuck.
  Eigen::Array<bool, Eigen::Dynamic, 1> motor_reverse =
      (joint_velocity * torque_command) < -0.001;
  is_joint_stuck_ += motor_reverse;

  is_finger_stuck_.setZero();
  if (is_joint_stuck_.segment<3>(0).all()) is_finger_stuck_(0) = true;
  if (is_joint_stuck_.segment<3>(3).all()) is_finger_stuck_(1) = true;
  if (is_joint_stuck_.segment<3>(6).all()) is_finger_stuck_(2) = true;
  if (is_joint_stuck_.segment<2>(9).all()) is_finger_stuck_(3) = true;

  if (motor_reverse.segment<3>(0).any()) is_finger_stuck_(0) = true;
  if (motor_reverse.segment<3>(3).any()) is_finger_stuck_(1) = true;
  if (motor_reverse.segment<3>(6).any()) is_finger_stuck_(2) = true;
  if (motor_reverse.segment<2>(9).any()) is_finger_stuck_(3) = true;
}

Eigen::VectorXd Robotiq3fHandMotionState::GraspJointPosition(
    int grasp_mode_index) const {
  Eigen::VectorXd position(robotiq_3f_num_joints_);
  position.setZero();
  // The numbers corresponds to the joint positions when the hand grasps a
  // medium size object, such as the mug. The final positions of the joints
  // are usually larger than the preset values, so that the fingers continuously
  // apply force on the object.

  // basic mode
  if (grasp_mode_index == 0)
    position << 0.478, 0.000, -0.506,
      0.446, 0.000, -0.471,
      0.495, 0.000, -0.524,
      0.000, 0.000;
  // wide mode
  else if (grasp_mode_index == 1)
    position << 0.487, 0.000, -0.515,
      0.438, 0.000, -0.463,
      0.495, 0.000, -0.524,
      0.1766, -0.1766;
  // pinch mode
  else if (grasp_mode_index == 2)
    position << 0.462, 0.000, -0.489,
      0.438, 0.000, -0.463,
      0.495, 0.000, -0.524,
      -0.1561, 0.1561;
  position.segment<2>(9) = PalmJointPositionGraspMode(grasp_mode_index);
  return position;
}

Eigen::VectorXd Robotiq3fHandMotionState::OpenJointPosition(
    int grasp_mode_index) const {
  Eigen::VectorXd position(robotiq_3f_num_joints_);
  // The preset position of the joints when the hand is open.
  position.setZero();
  position.segment<2>(9) = PalmJointPositionGraspMode(grasp_mode_index);
  return position;
}

Eigen::Vector2d Robotiq3fHandMotionState::PalmJointPositionGraspMode(
    int grasp_mode_index) const {
  Eigen::Vector2d palm_position;
  palm_position.setZero();
  // grasp_mode_index 0 is basic mode, joints are 0.
  if (grasp_mode_index == 1) {
    // wide - max is 0.250, 0.1766 on actual hand.
    palm_position(0) = 0.1766;
    palm_position(1) = -0.1766;
  } else if (grasp_mode_index == 2) {
    // pinch - min is -0.160, -0.1561 on actual hand.
    palm_position(0) = -0.1561;
    palm_position(1) = 0.1561;
  }
  return palm_position;
}

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
