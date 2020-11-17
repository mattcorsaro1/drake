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

#include "drake/examples/robotiq_3f/slider_lcm.h"

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

using systems::BasicVector;
using systems::Context;
using systems::DiscreteValues;
using systems::DiscreteUpdateEvent;

SliderCommandReceiver::SliderCommandReceiver() {
  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<lcmt_slider_command>{});
  state_output_port_ = this->DeclareVectorOutputPort(
      systems::BasicVector<double>(2),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, 0, 2, o);
      }).get_index();
  torque_output_port_ = this->DeclareVectorOutputPort(
      systems::BasicVector<double>(1),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, 2, 1, o);
      }).get_index();
  this->DeclarePeriodicDiscreteUpdate(kLcmStatusPeriod);
  // State + torque
  this->DeclareDiscreteState(3);
}

void SliderCommandReceiver::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>>& x) const {
  auto state_value = context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(x.size() == 1);
  state_value.setZero();
  state_value.head(1) = x;
}

void SliderCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& command = input->get_value<lcmt_slider_command>();

  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();
  // If we're using a default constructed message (haven't received
  // a command yet), keep using the initial state.
  if (command.num_joints != 0) {
    DRAKE_DEMAND(command.num_joints == 1);
    VectorX<double> new_positions(1);
    for (int i = 0; i < command.num_joints; ++i) {
      new_positions(i) = command.joint_position[i];
    }

    state_value.segment(1, 1).setZero();
    state_value.head(1) = new_positions;
  }

  // If the message does not contain torque commands, set torque command to
  // zeros.
  state_value.tail(1).setZero();
}

void SliderCommandReceiver::CopyStateToOutput(
    const Context<double>& context, int start_idx, int length,
    BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec =
      context.get_discrete_state(0).get_value().segment(start_idx, length);
}

SliderStatusSender::SliderStatusSender() {
  // Measured state.
  // 2 values: joint position and velocity
  state_input_port_ = this->DeclareInputPort(
                      systems::kVectorValued, 2).get_index();

  this->DeclareAbstractOutputPort(&SliderStatusSender::MakeOutputStatus,
                                  &SliderStatusSender::OutputStatus);
}

lcmt_slider_status SliderStatusSender::MakeOutputStatus() const {
  lcmt_slider_status msg{};
  // Just care about position
  msg.num_state_vals = 1;
  msg.slider_state.resize(msg.num_state_vals, 0);
  return msg;
}

void SliderStatusSender::OutputStatus(const Context<double>& context,
                                       lcmt_slider_status* output) const {
  lcmt_slider_status& status = *output;

  status.utime = context.get_time() * 1e6;

  // size 2?
  const systems::BasicVector<double>* state = this->EvalVectorInput(context,
    state_input_port_);

  status.slider_state[0] = state->GetAtIndex(0);
}

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
