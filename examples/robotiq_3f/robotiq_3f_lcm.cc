/// @file This file contains classes dealing with sending/receiving
/// LCM messages related to the robotiq_3f hand.
///
/// The core of the gripper logic is located in this file. The implementation
/// is based on ``Technical Report: Use of Hybrid Systems to model the
/// RobotiQ Adaptive Gripper'' by Giulia Franchi and Kris Hauser.
/// Each of the gripper's 3-jointed fingers is actuated by one motor and
/// wraps around objects it comes into contact with. These fingers can be
/// modeled as hybrid systems by sending a short series of joint commands,
/// and modifying the goals depending on when a link comes into contact
/// with an obstacle.
///
/// Currently, our LCM node receives a binary request to close the gripper.
/// We use joint angles and contact information to send and modify joint angle
/// goals. If contact is made and a joint in the physical underactuated hand
/// would stop, we stop the joint by setting a final goal some small value
/// above the current joint angle.
///
/// originally copied from examples/allegro_hand @ f646302

#include "drake/examples/robotiq_3f/robotiq_3f_lcm.h"

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

// Given a multibody::BodyIndex and vector that maps ordered bodies to ids,
// if the body_id corresponds to a gripper link (bodies 0-9 in our example),
// return the link's index. Otherwise, return the first index corresponding
// to a link not on the gripper (10).
int BodyIDtoLinkNum(multibody::BodyIndex body_id,
                    const std::vector<int> body_ids) {
  for (uint i=0; i<10; i++) {
    if (body_ids[i] == body_id) {
      return i;
    }
  }
  return 10;
}

// Given some PointPairContactInfo, if one of the bodies in the contact is
// the object with multibody::BodyIndex obj_id and the other is a finger link,
// return the ID of the link in contact.
int LinkInContact(const drake::multibody::PointPairContactInfo<double>& contact_info,
  const std::vector<int>& body_ids, int obj_id) {
  // bodyB is object
  if (contact_info.bodyB_index() == body_ids[obj_id]) {
    int link_num = BodyIDtoLinkNum(contact_info.bodyA_index(), body_ids);
    // in error case, returns object ID.
    if (link_num < obj_id)
    {
      return link_num;
    }
  }
  else if (contact_info.bodyA_index() == body_ids[obj_id]) {
    int link_num = BodyIDtoLinkNum(contact_info.bodyB_index(), body_ids);
    // in error case, returns obj ID.
    if (link_num < obj_id)
    {
      return link_num;
    }
  }
  // This contact is not between any finger links and the object, don't care
  return obj_id;
}

ContactProcessor::ContactProcessor(int num_links, bool track_collision)
    : num_links_(num_links), track_collision_(track_collision) {
  contact_input_port_ = this->DeclareAbstractInputPort(
            "contact_results_in",
            Value<multibody::ContactResults<double> >()).get_index();

  // Output is a vector with one value for each link.
  // If contact is made between the link and the object, the corresponding
  // element is set
  contact_history_output_port_ = this->DeclareVectorOutputPort(
      systems::BasicVector<double>(num_links_),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, 0, num_links_, o);
      }).get_index();
  this->DeclarePeriodicDiscreteUpdate(kLcmStatusPeriod);
  // State + torque
  this->DeclareDiscreteState(num_links_);
}

void ContactProcessor::initialize(
    Context<double>* context) const {
  auto state_value = context->get_mutable_discrete_state(0).get_mutable_value();
  state_value.setZero();
}

void ContactProcessor::CopyStateToOutput(
    const Context<double>& context, int start_idx, int length,
    BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec =
      context.get_discrete_state(0).get_value().segment(start_idx, length);
}

void ContactProcessor::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();

  // Latest contact results
  const multibody::ContactResults<double>& contact_results =
        this->get_input_port(contact_input_port_)
            .Eval<multibody::ContactResults<double> >(context);

  // If track_collision_ is true, initialize contact state with previous
  // state values - if a link had previously made contact,
  // its flag should be left up.
  VectorX<double> next_state(num_links_);
  for (int i=0; i<num_links_; i++) {
    if (track_collision_) {
      next_state[i] = state_value[i];
    }
    else {
      next_state[i] = 0;
    }
  }

  // Assume all contacts are point_pair and not hydroelastic because that's
  // what they were during tests.

  // Process each contact. If the magnitude exceeds a threshold and the contact
  // is between a link and an object, set the corresponding value in the
  // contact status vector.
  for (int contact_i=0; contact_i < contact_results.num_point_pair_contacts();
    contact_i++) {
    auto contact_info = contact_results.point_pair_contact_info(contact_i);
    auto contact_force = contact_info.contact_force();
    int link_in_contact = LinkInContact(contact_info, body_ids_, 10);
    if (link_in_contact < 10) {
      double contact_force_mag = std::sqrt(pow(contact_force[0], 2) + pow(contact_force[1], 2) + pow(contact_force[2], 2));
      if (contact_force_mag >= min_force_) {
        next_state[link_in_contact] = 1;
      }
    }
  }
  state_value.head(num_links_) = next_state;
}

// Receives binary commands to close the gripper
Robotiq3fCommandReceiver::Robotiq3fCommandReceiver(int num_joints, int num_links)
    : num_joints_(num_joints) {
  command_input_port_ = this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<lcmt_robotiq_3f_binary_command>{}).get_index();
  state_input_port_ = this->DeclareInputPort(
                      systems::kVectorValued, num_joints_ * 2).get_index();
  // 10 = num_links, num_joints <= 9
  contact_history_input_port_ = this->DeclareInputPort(
                      systems::kVectorValued, num_links).get_index();

  state_output_port_ = this->DeclareVectorOutputPort(
      systems::BasicVector<double>(num_joints_ * 2),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, 0, num_joints_ * 2, o);
      }).get_index();
  torque_output_port_ = this->DeclareVectorOutputPort(
      systems::BasicVector<double>(num_joints_),
      [this](const Context<double>& c, BasicVector<double>* o) {
        this->CopyStateToOutput(c, num_joints_ * 2, num_joints_, o);
      }).get_index();
  this->DeclarePeriodicDiscreteUpdate(kLcmStatusPeriod);
  // State + torque
  this->DeclareDiscreteState(num_joints_ * 3);
}

void Robotiq3fCommandReceiver::set_initial_position(
    Context<double>* context,
    const Eigen::Ref<const VectorX<double>>& x) const {
  auto state_value = context->get_mutable_discrete_state(0).get_mutable_value();
  DRAKE_ASSERT(x.size() == num_joints_);
  state_value.setZero();
  state_value.head(num_joints_) = x;
}

void Robotiq3fCommandReceiver::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input = this->EvalAbstractInput(context, command_input_port_);
  DRAKE_ASSERT(input != nullptr);
  const auto& binary_command = input->get_value<lcmt_robotiq_3f_binary_command>();

  // set joint goal state according to aforementioned technical report. If contact
  // is made, stop links and continue to wrap more distal links around object.
  BasicVector<double>& goal_state = discrete_state->get_mutable_vector(0);
  auto goal_state_value = goal_state.get_mutable_value();

  if (binary_command.close_hand) {
    // size 2*num_joints
    const systems::BasicVector<double>* state_vector =
      this->EvalVectorInput(context, state_input_port_);

    const systems::BasicVector<double>* contact_vector =
      this->EvalVectorInput(context, contact_history_input_port_);

    VectorX<double> new_goal(num_joints_);
    for (int j=0; j<num_joints_; j++) {
      new_goal[j] = goal_state_value(j);
    }

    // Wide or basic mode ====================================================
    if (num_joints_ == 9) {
      for (int finger=0; finger<3; finger++) {
        // None of the links have made contact
        if (not(contact_vector->GetAtIndex(finger*3) ||
                contact_vector->GetAtIndex(finger*3+1) ||
                contact_vector->GetAtIndex(finger*3+2))) {
          // 3 phases, according to Technical Report: Use of Hybrid Systems to model the RobotiQ Adaptive Gripper:
          // 0 <= g <= 110 (goal for joint 1 is first to be set)
          if (std::abs(goal_state_value(finger*3) - 0) < comparison_theta_) {
            new_goal[finger*3] = 0.959931;
            new_goal[finger*3+1] = 0;
            new_goal[finger*3+2] = -0.959931;
          }
          // At goal set in previous step, move on to next phase
          // 110 < g <= 140
          else if (std::abs(state_vector->GetAtIndex(finger*3)-
                   0.959931) < comparison_theta_) {
            new_goal[finger*3] = j1_max;
          }
          // 140 < g <= 240
          else if (std::abs(state_vector->GetAtIndex(finger*3)-
                   j1_max) < comparison_theta_) {
            new_goal[finger*3+1] = j2_max;
          }
        }
        // link 3 in contact
        else if (contact_vector->GetAtIndex(finger*3+2)) {
          // Stop all
          new_goal[finger*3] = std::min(j1_max, state_vector->GetAtIndex(finger*3)+additional_goal_*1.2);
          new_goal[finger*3+1] = std::min(j2_max, state_vector->GetAtIndex(finger*3+1)+additional_goal_);
          new_goal[finger*3+2] = std::min(j3_max, state_vector->GetAtIndex(finger*3+2)+additional_goal_);
        }
        // link 2 in contact
        else if (contact_vector->GetAtIndex(finger*3+1)) {
          // stop 1 & 2, close 3
          new_goal[finger*3] = std::min(j1_max, state_vector->GetAtIndex(finger*3)+additional_goal_);
          new_goal[finger*3+1] = std::min(j2_max, state_vector->GetAtIndex(finger*3+1)+additional_goal_);
          new_goal[finger*3+2] = j3_max;
        }
        // link 1 in contact
        else if (contact_vector->GetAtIndex(finger*3)) {
          // stop 1, close 2, close 3
          new_goal[finger*3] = std::min(j1_max, state_vector->GetAtIndex(finger*3)+additional_goal_);
          new_goal[finger*3+1] = j2_max;
          new_goal[finger*3+2] = j3_max;
        }
      }
    }
    // pincher mode
    else if (num_joints_ == 6) {
      // fingers 1 and 2 should be kept together at all times

      bool pincher_in_contact[3];
      for (int finger=0; finger<3; finger++) {
        pincher_in_contact[finger] = contact_vector->GetAtIndex(finger*3) ||
                                contact_vector->GetAtIndex(finger*3+1) ||
                                contact_vector->GetAtIndex(finger*3+2);
      }
      // If sufficient force made on both sides of parallel jaw
      if ((pincher_in_contact[1] || pincher_in_contact[2]) && pincher_in_contact[0])
      {
        // stop everything
        for (int finger=0; finger<3; finger++) {
          new_goal[finger*2] = std::min(0.959931, state_vector->GetAtIndex(finger*2)+additional_goal_);
          new_goal[finger*2+1] = std::max(-0.959931, state_vector->GetAtIndex(finger*2+1)-additional_goal_);
        }
      }
      else {
        for (int finger=0; finger<3; finger++) {
          new_goal[finger*2] = 0.959931;//0.9246;
          new_goal[finger*2+1] = -0.959931;
        }
      }
    }
    else {
      // TODO(mcorsaro): make this an assert
      std::cerr << "Expected 6 or 9 joints, got " << num_joints_ << std::endl;
      std::exit(EXIT_FAILURE);
    }
    goal_state_value.head(num_joints_) = new_goal;
  }

  // Goal velocity 0
  goal_state_value.segment(num_joints_, num_joints_).setZero();
  // Goal torque 0
  goal_state_value.tail(num_joints_).setZero();

}

void Robotiq3fCommandReceiver::CopyStateToOutput(
    const Context<double>& context, int start_idx, int length,
    BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec =
      context.get_discrete_state(0).get_value().segment(start_idx, length);
}

Robotiq3fStatusSender::Robotiq3fStatusSender(int num_joints)
    : num_joints_(num_joints) {
  // Commanded state.
  command_input_port_ = this->DeclareInputPort(
                        systems::kVectorValued, num_joints_ * 2).get_index();
  // Measured state.
  state_input_port_ = this->DeclareInputPort(
                      systems::kVectorValued, num_joints_ * 2).get_index();
  contact_input_port_ = this->DeclareAbstractInputPort(
            "contact_results_in",
            Value<multibody::ContactResults<double> >()).get_index();
  // Commanded torque.
  command_torque_input_port_ = this->DeclareInputPort(
                              systems::kVectorValued, num_joints_).get_index();

  this->DeclareAbstractOutputPort(&Robotiq3fStatusSender::MakeOutputStatus,
                                  &Robotiq3fStatusSender::OutputStatus);
}

lcmt_robotiq_3f_status Robotiq3fStatusSender::MakeOutputStatus() const {
  lcmt_robotiq_3f_status msg{};
  msg.num_joints = num_joints_;
  // 3 per finger plus palm
  msg.num_links = 10;
  msg.joint_position_measured.resize(msg.num_joints, 0);
  msg.joint_velocity_estimated.resize(msg.num_joints, 0);
  msg.joint_position_commanded.resize(msg.num_joints, 0);
  msg.joint_torque_commanded.resize(msg.num_joints, 0);
  msg.child_link_in_contact.resize(msg.num_links, false);
  msg.contact_with_environment = false;
  return msg;
}

void Robotiq3fStatusSender::OutputStatus(const Context<double>& context,
                                       lcmt_robotiq_3f_status* output) const {
  lcmt_robotiq_3f_status& status = *output;

  status.utime = context.get_time() * 1e6;

  // size 2*num_joints
  const systems::BasicVector<double>* command =
      this->EvalVectorInput(context, command_input_port_);

  // size 2*num_joints
  const systems::BasicVector<double>* state = this->EvalVectorInput(context,
    state_input_port_);

  // size 1*num_joints
  const systems::BasicVector<double>* commanded_torque =
      this->EvalVectorInput(context, command_torque_input_port_);

  const multibody::ContactResults<double>& contact_results =
        this->get_input_port(contact_input_port_)
            .Eval<multibody::ContactResults<double> >(context);
  // Assume all contacts are point_pair and not hydroelastic because that's
  // what they were with the mug test.
  for (int contact_i=0; contact_i < contact_results.num_point_pair_contacts();
    contact_i++) {
    auto contact_info = contact_results.point_pair_contact_info(contact_i);
    int link_in_contact = LinkInContact(contact_info, body_ids_, 10);
    if (link_in_contact < 10) {
      status.child_link_in_contact[link_in_contact] = true;
    }
    int link_in_contact_with_env = LinkInContact(contact_info, body_ids_, 11);
    if (link_in_contact_with_env < 10) {
      std::cout << link_in_contact_with_env << std::endl;
      status.contact_with_environment = true;
    }
  }

  for (int i = 0; i < num_joints_; ++i) {
    status.joint_position_measured[i] = state->GetAtIndex(i);
    status.joint_velocity_estimated[i] = state->GetAtIndex(i + num_joints_);
    status.joint_position_commanded[i] = command->GetAtIndex(i);
    status.joint_torque_commanded[i] = commanded_torque->GetAtIndex(i);
  }
}

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
