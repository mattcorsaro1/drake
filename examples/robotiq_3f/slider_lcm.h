#pragma once

/// @file This file contains classes dealing with sending
/// LCM messages related to the slider.
///
/// The slider is a robot with no visual or collision elements and a single
/// prismatic joint that moves along the positive z axis.
///
/// In our example, the robotiq_3f gripper is fixed to the slider, and
/// after executing a grasp, the slider moved the gripper upwards
///
/// originally copied from examples/allegro_hand @ f646302
/// modified from robotiq_3f_lcm

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/robotiq_3f/robotiq_3f_common.h"
#include "drake/lcmt_slider_command.hpp"
#include "drake/lcmt_slider_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

/// Handles lcmt_slider_command messages from a LcmSubscriberSystem.
/// Has two output ports: one for the commanded position for the joint along
/// with a zero velocity for the joint, and another for commanded additional
/// feedforward joint torque. The joint torque command is currently not used.
class SliderCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SliderCommandReceiver)

  explicit SliderCommandReceiver();

  /// Sets the initial position of the controlled slider prior to any
  /// commands being received.  @param x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.
  void set_initial_position(systems::Context<double>* context,
                            const Eigen::Ref<const VectorX<double>>& x) const;

  const systems::OutputPort<double>& get_commanded_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPort<double>& get_commanded_torque_output_port() const {
    return this->get_output_port(torque_output_port_);
  }

 private:
  void CopyStateToOutput(const systems::Context<double>& context, int start_idx,
                         int length,
                         systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  int state_output_port_ = 0;
  int torque_output_port_ = 0;
};

class SliderStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SliderStatusSender)

  explicit SliderStatusSender();

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

 private:
  // This is the method to use for the output port allocator.
  lcmt_slider_status MakeOutputStatus() const;

  // This is the method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_slider_status* output) const;

  int state_input_port_ = 0;

};

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
