#pragma once

/// @file LCM system that takes in an object's state (pose and velocity)
/// and outputs position.
///
/// originally copied from examples/allegro_hand @ f646302
/// modified from robotiq_3f_lcm

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_obj_status.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

/// Creates and outputs lcmt_obj_status messages.
///
/// Used to subscribe to a given object's pose.
/// The object is specified when connecting the plant's state output port
/// to this status sender.
///
/// By default, the state consists of 13 values - 7 for position, 6 for vel.
/// We just care about the position.
///
/// In practice, we get the object's initial position, then read the position
/// again after a grasp attempt to determine if the object was lifted.
///
/// This system is presently only used in simulation.
class ObjStatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ObjStatusSender)

  explicit ObjStatusSender();

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

 private:
  // This is the method to use for the output port allocator.
  lcmt_obj_status MakeOutputStatus() const;

  // This is the calculator method for the output port.
  void OutputStatus(const systems::Context<double>& context,
                    lcmt_obj_status* output) const;

  int state_input_port_ = 0;

};

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
