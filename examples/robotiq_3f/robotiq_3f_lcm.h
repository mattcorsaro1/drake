#pragma once

/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the robotiq_3f hand.
///
/// originally copied from examples/allegro_hand @ f646302

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/robotiq_3f/robotiq_3f_common.h"
#include "drake/lcmt_robotiq_3f_command.hpp"
#include "drake/lcmt_robotiq_3f_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

// The publication period of hand status publish.
// TODO(WenzhenYuan-TRI): match the value with the real hand's communication
// rate.
const double kLcmStatusPeriod = 0.003;

/// Handles lcmt_robotiq_3f_command messages from a LcmSubscriberSystem.
/// Has two output ports: one for the commanded position for each joint along
/// with a zero velocity for each joint, and another for commanded additional
/// feedforward joint torque. The joint torque command is currently not used.
class Robotiq3fCommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Robotiq3fCommandReceiver)

  explicit Robotiq3fCommandReceiver(int num_joints = kRobotiq3fNumJoints);

  /// Sets the initial position of the controlled hand prior to any
  /// commands being received.  @param x contains the starting position.
  /// This position will be the commanded position (with zero
  /// velocity) until a position message is received.  If this
  /// function is not called, the open hand pose will be the zero
  /// configuration.
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
  const int num_joints_ = 11;
};

/// Creates and outputs lcmt_robotiq_3f_status messages.
///
/// This system has three vector-valued input ports, one for the plant's
/// current state, one for the most recently received position command, and one
/// for the most recently received joint torque command.
/// The state and command ports contain a position and velocity for each joint,
/// which is supposed to be in the order of
/// middle(3)-finger1(3)-finger2(3)-palm(2).
///
/// This system has one abstract valued output port that contains a
/// Value object templated on type `lcmt_robotiq_3f_status`. Note that
/// this system does not actually send this message on an LCM channel. To send
/// the message, the output of this system should be connected to an input port
/// of a systems::lcm::LcmPublisherSystem that accepts a Value object
/// templated on type `lcmt_robotiq_3f_status`.
///
/// This system is presently only used in simulation.
class Robotiq3fStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Robotiq3fStatusSender)

  explicit Robotiq3fStatusSender(int num_joints = kRobotiq3fNumJoints);

  const systems::InputPort<double>& get_command_input_port() const {
    return this->get_input_port(command_input_port_);
  }

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const systems::InputPort<double>& get_commanded_torque_input_port() const {
    return this->get_input_port(command_torque_input_port_);
  }

 private:
  // This is the method to use for the output port allocator.
  lcmt_robotiq_3f_status MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_robotiq_3f_status* output) const;

  int command_input_port_ = 0;
  int state_input_port_ = 0;
  int command_torque_input_port_ = 0;

  const int num_joints_ = 11;
};

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
