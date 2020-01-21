// originally copied from examples/allegro_hand @ f646302

#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_robotiq_3f_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

constexpr int kRobotiq3fNumJoints = 11;

/// Set the feedback gains for the simulated position control
void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                Eigen::VectorXd* Kd);

/// Creates selector matrices which extract state xₛ in a known order from the
/// plant's full x (`xₛ = Sx⋅x`) and promote the controller's ordered yₛ into
/// the full plant's input actuation (`u = Su⋅uₛ`).
/// The matrices are used to initialize the PID controller for the hand.
/// @see MultibodyPlant::MakeStateSelectorMatrix(),
/// MultibodyPlant::MakeActuatorSelectorMatrix() for detailed definitions for
/// the selector matrices.
/// @see systems::controllers::PidController for documentation on how these
/// selector matrices are used in the PID controller.
/// @param Sx the matrix to match the output state of the plant into the state
/// of the finger joints in the desired order.
/// @param Sy the matrix to match the output torque for the hand joint
/// actuators in the desired order into the input actuation of the plant.
void GetControlPortMapping(
    const multibody::MultibodyPlant<double>& plant,
    MatrixX<double>* Sx, MatrixX<double>* Sy);

/// Defines the desired ordering of the finger joints by name. The fingers are
/// ordered as [finger_1, finger_2, finger_middle, palm_finger_1, palm_finger_2]
/// and the joints of each finger are
/// ordered from most proximal to most distal (relative to the palm).
std::vector<std::string> GetPreferredJointOrdering();

/// Detecting the state of the fingers: whether the joints are moving, or
/// reached the destination, or got stuck by external collisions in the midway.
/// The class uses only the hand status from the MBP as the input, and calculate
/// the state according to the position, velocity, and command position of each
/// joint. The class also contains two commonly used pre-set joint state for
/// opening the hand and closing the hand for grasping.
class Robotiq3fHandMotionState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Robotiq3fHandMotionState)

  Robotiq3fHandMotionState();

  /// Update the states of the joints and fingers upon receiving the new
  /// message about hand states.
  void Update(const lcmt_robotiq_3f_status& robotiq_3f_state_msg);

  /// Pre-set joint positions for grasping objects and open hands
  /// in the 3 finger modes (basic, wide, pinch).
  /// @param grasp_mode_index the index corresponding to the grasp mode
  /// to set the hand to - [basic, wide, pinch]

  Eigen::VectorXd GraspJointPosition(int grasp_mode_index) const;
  Eigen::VectorXd OpenJointPosition(int grasp_mode_index) const;

  /// Helper function that returns two joint positions corresponding to
  /// the two palm joints. These values affect the grasp mode.
  /// @param grasp_mode_index the index corresponding to the grasp mode
  /// to set the hand to - [basic, wide, pinch]

  Eigen::Vector2d PalmJointPositionGraspMode(int grasp_mode_index) const;

  /// Returns true when the finger is stuck, which means the joints on the
  /// finger stops moving or back driving, regardless of it having reached the
  /// target position or not.
  bool IsFingerStuck(int finger_index) const {
    return is_finger_stuck_(finger_index);
  }
  bool IsAllFingersStuck() const { return is_finger_stuck_.all(); }

 private:
  int robotiq_3f_num_joints_{kRobotiq3fNumJoints};
  int finger_num_{0};

  Eigen::Array<bool, Eigen::Dynamic, 1> is_joint_stuck_;
  Eigen::Array<bool, Eigen::Dynamic, 1> is_finger_stuck_;

  /// The velocity threshold under which the joint is considered not moving.
  static const double velocity_thresh_;
};

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
