/// @file LCM system that takes in an object's state (pose and velocity)
/// and outputs position.
///
/// originally copied from examples/allegro_hand @ f646302
/// modified from robotiq_3f_lcm

#include "drake/examples/robotiq_3f/obj_status_lcm.h"

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

ObjStatusSender::ObjStatusSender() {
  // Measured state.
  state_input_port_ = this->DeclareInputPort(
                      systems::kVectorValued, 13).get_index();

  this->DeclareAbstractOutputPort(&ObjStatusSender::MakeOutputStatus,
                                  &ObjStatusSender::OutputStatus);
}

lcmt_obj_status ObjStatusSender::MakeOutputStatus() const {
  lcmt_obj_status msg{};
  msg.num_state_vals = 3;
  msg.obj_state.resize(msg.num_state_vals, 0);
  return msg;
}

void ObjStatusSender::OutputStatus(const Context<double>& context,
                                       lcmt_obj_status* output) const {
  lcmt_obj_status& status = *output;

  status.utime = context.get_time() * 1e6;

  // size 13. Orientation quaternion (4), position vector (3), then velocity (6)
  const systems::BasicVector<double>* state = this->EvalVectorInput(context,
    state_input_port_);

  // Copy position state values (4-6, inclusive) into message
  for (int i = 4; i < 4+status.num_state_vals; ++i) {
    status.obj_state[i-4] = state->GetAtIndex(i);
  }
}

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
