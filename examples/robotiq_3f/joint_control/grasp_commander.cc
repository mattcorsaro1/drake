/// @file
///
/// originally copied from examples/allegro_hand @ f646302

#include <chrono>
#include <thread>

#include <Eigen/Dense>
#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"
#include <math.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/robotiq_3f/slider_lcm.h"
#include "drake/examples/robotiq_3f/obj_status_lcm.h"
#include "drake/examples/robotiq_3f/robotiq_3f_lcm.h"
#include "drake/lcmt_slider_command.hpp"
#include "drake/lcmt_slider_status.hpp"
#include "drake/lcmt_obj_status.hpp"
#include "drake/lcmt_robotiq_3f_binary_command.hpp"
#include "drake/lcmt_robotiq_3f_status.hpp"

namespace drake {
namespace examples {
namespace robotiq_3f {
namespace {

const char* const kLcmStatusChannel = "ROBOTIQ3F_STATUS";
const char* const kLcmBinaryCommandChannel = "ROBOTIQ3F_BINARY_COMMAND";
const char* const kLcmSliderCommandChannel = "SLIDER_COMMAND";
const char* const kLcmSliderStatusChannel = "SLIDER_STATUS";
const char* const kLcmObjStatusChannel = "OBJ_STATUS";

// While waiting for an updated LCM message, handleTimeout this many times
const int _num_wait_cycles = 60;
// Time to wait, in seconds, while closing hand if fingers are still moving
const double _max_closing_time = 3;
// Time to wait, in seconds, for slider to move
const double _max_slider_time = _max_closing_time+3;
// Height to raise hand along slider's z axis
const double _goal_height = 0.1;

// single LCM object for all classes to interact with
::lcm::LCM lcm_;

// returns true if velocity of all links is below threshold
bool isAllFingersStuck(const lcmt_robotiq_3f_status& robotiq_3f_state_msg,
  double velocity_thresh=0.01) {

  int num_joints = robotiq_3f_state_msg.num_joints;
  int num_fingers = 3;
  int num_joints_per_finger = num_joints/num_fingers;

  const double* ptr = &(robotiq_3f_state_msg.joint_velocity_estimated[0]);
  const Eigen::ArrayXd joint_velocity =
    Eigen::Map<const Eigen::ArrayXd>(ptr, num_joints);
  Eigen::Array<bool, Eigen::Dynamic, 1> is_joint_stuck =
    joint_velocity.abs() < velocity_thresh;

  for (int i=0; i<num_fingers; i++) {
    // if one finger is not stuck, return false
    if (!(is_joint_stuck.segment(i*num_joints_per_finger,
                                 num_joints_per_finger).all())) {
      return false;
    }
  }
  // checked all fingers and all were stuck, return true
  return true;
}

// Gripper is attached to slider with no collision elements that moves upward
// in the positive z direction after completing a grasp
class SliderCommander {
 public:
  SliderCommander() {
    lcm_.subscribe(kLcmSliderStatusChannel, &SliderCommander::HandleStatus,
      this);
    slider_status_.utime = 0;
    slider_status_.num_state_vals = 1;
    slider_status_.slider_state.resize(slider_status_.num_state_vals, 0);
  }

  void DropSlider() {
    MoveSliderToPosition(0.0);
  }

  void RaiseSlider() {
    MoveSliderToPosition(_goal_height);
  }

  // return true if the slider reaches the goal height within the time limit,
  // false otherwise
  bool WaitWhileSliderBelowHeight(double slider_goal_height) {
    double utime_started = slider_status_.utime;
    double time_elapsed = 0;
    // wait until the slider moves above the specified height or times out
    while (slider_status_.slider_state[0] < slider_goal_height &&
           time_elapsed*1e-6 < _max_slider_time) {
      while (0 == lcm_.handleTimeout(10) || slider_status_.utime == -1) {}
      time_elapsed = slider_status_.utime - utime_started;
    }
    if (time_elapsed*1e-6 >= _max_slider_time) {
      return false;
    }
    return true;
  }

  double GetSliderHeight() {
    for (int i = 0; i < _num_wait_cycles; i++) {
      while (0 == lcm_.handleTimeout(10)) {}
    }
    return slider_status_.slider_state[0];
  }
 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_slider_status* status) {
    slider_status_ = *status;
  }

  void MoveSliderToPosition(double position) {
    slider_command_.num_joints = 1;
    slider_command_.joint_position.resize(1, position);
    lcm_.publish(kLcmSliderCommandChannel, &slider_command_);
  }

  lcmt_slider_command slider_command_;
  lcmt_slider_status slider_status_;
};

// Receives and processes object pose
class ObjStatusListener {
 public:
  ObjStatusListener() {
    lcm_.subscribe(kLcmObjStatusChannel, &ObjStatusListener::HandleStatus,
                   this);
    latest_position_.resize(state_size_, 0);
  }

  double GetObjHeight() {
    for (int i = 0; i < _num_wait_cycles; i++) {
      while (0 == lcm_.handleTimeout(10)) {}
    }
    return latest_obj_height_;
  }

 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_obj_status* status) {
    obj_status_ = *status;
    latest_position_.resize(state_size_, 0);

    DRAKE_ASSERT(obj_status_.num_state_vals == state_size_);

    for (int i=0; i<obj_status_.num_state_vals; i++) {
      latest_position_[i] = obj_status_.obj_state[i];
    }
    DRAKE_ASSERT(state_size_ >= 3);
    latest_obj_height_ = latest_position_[2];
  }

  lcmt_obj_status obj_status_;
  const int state_size_ = 3;
  double latest_obj_height_;
  std::vector<double> latest_position_;
};

class HandCommander {
 public:
  HandCommander() {
    lcm_.subscribe(kLcmStatusChannel, &HandCommander::HandleStatus,
                   this);
  }

  // return true if contact is made between the table and any link
  bool contactWithTable() {
    for (int i = 0; i < _num_wait_cycles; i++) {
      while (0 == lcm_.handleTimeout(10)) {}
    }
    return table_contact_;
  }

  // return true if contact is made between the object and any link
  bool contactWithObj() {
    for (int i = 0; i < _num_wait_cycles; i++) {
      while (0 == lcm_.handleTimeout(10)) {}
    }
    for (uint i = 0; i < robotiq_3f_status_.child_link_in_contact.size(); i++) {
      if (robotiq_3f_status_.child_link_in_contact[i] > 0) {
        return true;
      }
    }
    return false;
  }

  // given two vectors of equal length, return the maximum absolute difference
  // between values at corresponding indices
  double MaxDoubleDiffAbs(const std::vector<double>& v1,
                          const std::vector<double>& v2) {
    DRAKE_ASSERT(v1.size() == v2.size());
    double max_diff = 0;
    for (uint i=0; i<v1.size(); i++)
    {
      double diff = fabs(v1[i]-v2[i]);
      if (diff > max_diff) {
        max_diff = diff;
      }
    }
    return max_diff;
  }

  bool grasp() {
    robotiq_3f_binary_command_.close_hand = false;
    flag_moving_ = true;
    table_contact_ = false;
    // before initiating grasp, make sure no contact is made between the gripper
    // and table and between the gripper and object in the initial configuration
    if (contactWithTable() || contactWithObj()) {
      return false;
    }

    CloseHand();

    if (contactWithTable()) {
      return false;
    }
    else {
      return true;
    }
  }

 private:
  void CloseHand() {
    PublishBinaryCommand(true);
  }

  void OpenHand() {
    PublishBinaryCommand(false);
  }

  inline void PublishBinaryCommand(bool close_hand) {
    std::vector<double> initial_joint_state =
      robotiq_3f_status_.joint_position_measured;
    robotiq_3f_binary_command_.close_hand = close_hand;
    lcm_.publish(kLcmBinaryCommandChannel, &robotiq_3f_binary_command_);
    //
    // A time delay at the initial moving stage so that the noisy data from the
    // hand motion is filtered.
    for (int i = 0; i < _num_wait_cycles; i++) {
      while (0 == lcm_.handleTimeout(10) || robotiq_3f_status_.utime == -1) {
      }
    }
    double utime_started = robotiq_3f_status_.utime;
    double time_elapsed = 0;
    // wait until the fingers are stuck, or stop moving.
    // because flag_moving may be zero initially, wait until at least one joint
    // has moved past a small threshold
    while ((flag_moving_ ||
            MaxDoubleDiffAbs(robotiq_3f_status_.joint_position_measured,
                             initial_joint_state) < 0.1) &&
            time_elapsed*1e-6 < _max_closing_time) {
      while (0 == lcm_.handleTimeout(10) || robotiq_3f_status_.utime == -1) {
      }
      time_elapsed = robotiq_3f_status_.utime - utime_started;
    }
  }

  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_robotiq_3f_status* status) {
    robotiq_3f_status_ = *status;
    flag_moving_ = !isAllFingersStuck(robotiq_3f_status_);
    if (robotiq_3f_status_.contact_with_environment) {
      table_contact_ = true;
    }
  }

  lcmt_robotiq_3f_status robotiq_3f_status_;
  lcmt_robotiq_3f_binary_command robotiq_3f_binary_command_;

  bool flag_moving_ = true;
  bool table_contact_ = false;
};

int grasp_and_lift_object() {
  HandCommander hand_runner;
  SliderCommander slider_runner;
  ObjStatusListener obj_listener;

  // Close the hand. Returns false if the gripper is in contact with the 
  // object before executing the grasp, or if the gripper is in contact with
  // the table surface before or after execution
  bool grasp_executed = hand_runner.grasp();

  if (grasp_executed) {
    double initial_height = obj_listener.GetObjHeight();
    slider_runner.RaiseSlider();

    // wait while the slider raises, check object height once it's almost at
    // the goal.
    slider_runner.WaitWhileSliderBelowHeight(_goal_height-0.01);

    double final_height = obj_listener.GetObjHeight();
    double height_change = final_height - initial_height;

    std::cout << "Height changed by " << height_change*100 << " cm." <<
      std::endl;
  }
  else {
    std::cout << "Gripper made contact with object before executing grasp, " <<
      "or contact with table before or after grasp." << std::endl;
  }

  return 0;
}

}  // namespace
}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Controller to close hand, move hand, and check object height "
      "to determine if a grasp was successful.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::robotiq_3f::grasp_and_lift_object();
}
