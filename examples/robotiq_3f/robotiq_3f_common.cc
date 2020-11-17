/// originally copied from examples/allegro_hand @ f646302

#include "drake/examples/robotiq_3f/robotiq_3f_common.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

const double Robotiq3fHandMotionState::velocity_thresh_ = 0.03;

using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;

// TODO(mcorsaro): These values result in visually appropriate behavior, but
// do not correspond to the torques output by the physical gripper.
void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                Eigen::VectorXd* Kd, int num_joints) {
  *Kp = Eigen::VectorXd::Ones(num_joints) * 5e-1;
  *Kd = Eigen::VectorXd::Constant(Kp->size(), 4e-2);
  *Ki = Eigen::VectorXd::Constant(Kp->size(), 0);

  // increase gains for joint 1 on each finger
  int num_fingers = 3;
  // Joint 1 is 0, 3, 6 if 9 joints articulated; 0, 2, 4 if 6 (pincher mode)
  for (int joint_1_index=0; joint_1_index<num_joints; joint_1_index += num_joints/num_fingers) {
    (*Kp)[joint_1_index] *= 1.2;
    (*Kd)[joint_1_index] *= 1.2;
    (*Ki)[joint_1_index] *= 1.2;
  }
}

std::vector<std::string> GetPreferredJointOrdering(int num_joints) {
  std::vector<std::string> joint_name_mapping;

  // finger_middle - index 0
  joint_name_mapping.push_back("finger_middle_joint_1");
  if (num_joints == 9) {
    joint_name_mapping.push_back("finger_middle_joint_2");
  }
  joint_name_mapping.push_back("finger_middle_joint_3");

  // finger_1 - index 1
  joint_name_mapping.push_back("finger_1_joint_1");
  if (num_joints == 9) {
    joint_name_mapping.push_back("finger_1_joint_2");
  }
  joint_name_mapping.push_back("finger_1_joint_3");

  // finger_2 - index 2
  joint_name_mapping.push_back("finger_2_joint_1");
  if (num_joints == 9) {
    joint_name_mapping.push_back("finger_2_joint_2");
  }
  joint_name_mapping.push_back("finger_2_joint_3");

  /*
  // palm - index 3
  joint_name_mapping.push_back("palm_finger_1_joint");
  joint_name_mapping.push_back("palm_finger_2_joint");
  */

  return joint_name_mapping;
}

void GetControlPortMapping(
    const MultibodyPlant<double>& plant,
    MatrixX<double>* Sx, MatrixX<double>* Sy, int num_joints) {
  // Retrieve the list of finger joints in a user-defined ordering.
  const std::vector<std::string> joints_in_preferred_order =
      GetPreferredJointOrdering(num_joints);

  // Make a list of the same joints but by JointIndex.
  std::vector<JointIndex> joint_index_mapping;
  for (const auto& joint_name : joints_in_preferred_order) {
    joint_index_mapping.push_back(plant.GetJointByName(joint_name).index());
  }

  *Sx = plant.MakeStateSelectorMatrix(joint_index_mapping);
  *Sy = plant.MakeActuatorSelectorMatrix(joint_index_mapping);
}

Robotiq3fHandMotionState::Robotiq3fHandMotionState(int num_joints)
    : finger_num_(3),
      num_joints_(num_joints),
      is_joint_stuck_(num_joints_),
      is_finger_stuck_(finger_num_) {
        num_joints_per_finger_ = num_joints_/finger_num_;
      }

void Robotiq3fHandMotionState::Update(
    const lcmt_robotiq_3f_status& robotiq_3f_state_msg) {
  const lcmt_robotiq_3f_status status = robotiq_3f_state_msg;

  const double* ptr = &(status.joint_velocity_estimated[0]);
  const Eigen::ArrayXd joint_velocity =
      Eigen::Map<const Eigen::ArrayXd>(ptr, num_joints_);
  const Eigen::ArrayXd torque_command = Eigen::Map<const Eigen::ArrayXd>(
      &(status.joint_torque_commanded[0]), num_joints_);

  is_finger_stuck_.setZero();
  for (int i=0; i<finger_num_; i++) {
    // Block containing n elements, starting at position i
    if (is_joint_stuck_.segment(i*num_joints_per_finger_, num_joints_per_finger_).all()) {
      is_finger_stuck_(i) = true;
    }
  }
}

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
