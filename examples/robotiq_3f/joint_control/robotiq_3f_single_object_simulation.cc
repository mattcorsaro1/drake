/// @file
///
/// originally copied from examples/allegro_hand @ f646302

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/robotiq_3f/obj_status_lcm.h"
#include "drake/examples/robotiq_3f/robotiq_3f_common.h"
#include "drake/examples/robotiq_3f/robotiq_3f_lcm.h"
#include "drake/examples/robotiq_3f/slider_lcm.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/lcmt_obj_status.hpp"
#include "drake/lcmt_robotiq_3f_binary_command.hpp"
#include "drake/lcmt_robotiq_3f_status.hpp"
#include "drake/lcmt_slider_command.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace robotiq_3f {
namespace {

using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::MultibodyPlant;

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds");
DEFINE_double(max_time_step, 1.5e-4,
              "Simulation time step used for intergrator.");
DEFINE_bool(remove_gravity, false,
            "Whether adding gravity (9.81 m/s^2) in the simulation");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_string(grasp_type, "basic_power", "Supported grasp types are "
              "wide_power, wide_precision, basic_power, basic_precision, and "
              "pincher. Grasp type is parameterized by gripper operating mode "
              "(one of three discrete pre-grasp finger configurations) and "
              "distance from the palm to the object.");
DEFINE_string(obj, "cylinder", "Object name (box or cylinder)");

// Gripper pose. If default values are left unchanged, sets pose based on
// specified grasp type.
// Position
DEFINE_double(gripper_x, 0, "gripper x position");
DEFINE_double(gripper_y, 0, "gripper y position");
DEFINE_double(gripper_z, 0, "gripper z position");
// Orientation
DEFINE_double(gripper_qw, 1, "gripper w quaternion orientation");
DEFINE_double(gripper_qx, 0, "gripper x quaternion orientation");
DEFINE_double(gripper_qy, 0, "gripper y quaternion orientation");
DEFINE_double(gripper_qz, 0, "gripper z quaternion orientation");

void DoMain() {
  DRAKE_DEMAND(FLAGS_grasp_type == "wide_power" ||
               FLAGS_grasp_type == "wide_precision" ||
               FLAGS_grasp_type == "basic_power" ||
               FLAGS_grasp_type == "basic_precision" ||
               FLAGS_grasp_type == "pincher");

  DRAKE_DEMAND(FLAGS_obj == "box" || FLAGS_obj == "cylinder");

  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
    &builder, FLAGS_max_time_step);
  scene_graph.set_name("scene_graph");

  // Number of joints, used in gripper mode.
  // Currently have separate URDFs for each mode so fingers stay in place.
  // 3 joints per finger in wide and basic mode, but with pincher mode,
  // joint between links 1 and 2 remains fixed
  uint num_joints = 0;
  if (FLAGS_grasp_type == "pincher") {
    num_joints = 6;
  }
  else {
    num_joints = 9;
  }

  double gripper_x, gripper_y, gripper_z;
  double gripper_qw, gripper_qx, gripper_qy, gripper_qz;

  // extract gripper mode from grasp type: first substring before '_'
  std::string gripper_mode = FLAGS_grasp_type.substr(0,
                                                FLAGS_grasp_type.find('_'));

  // if any of the pose parameters are not their default values, use
  // all variables to set pose instead of grasp type
  if (FLAGS_gripper_x != 0 || FLAGS_gripper_y != 0 || FLAGS_gripper_z != 0 ||
    FLAGS_gripper_qw != 1 ||
    FLAGS_gripper_qx != 0 || FLAGS_gripper_qy != 0 || FLAGS_gripper_qz != 0) {
    gripper_x = FLAGS_gripper_x;
    gripper_y = FLAGS_gripper_y;
    gripper_z = FLAGS_gripper_z;
    gripper_qw = FLAGS_gripper_qw;
    gripper_qx = FLAGS_gripper_qx;
    gripper_qy = FLAGS_gripper_qy;
    gripper_qz = FLAGS_gripper_qz;
  }
  else {
    // default pose values were unchanged, set pose based on grasp type
    if (FLAGS_obj == "box") {
      if (FLAGS_grasp_type == "wide_power" ||
          FLAGS_grasp_type == "basic_power") {
        gripper_x = 0;
        gripper_y = 0;
        gripper_z = 0.2;
        gripper_qw = 0.7071;
        gripper_qx = -0.7071;
        gripper_qy = 0;
        gripper_qz = 0;
      }
      else {
        gripper_x = 0;
        gripper_y = 0;
        gripper_z = 0.27;
        gripper_qw = 0.7071;
        gripper_qx = -0.7071;
        gripper_qy = 0;
        gripper_qz = 0;
      }
    }
    else {
      if (FLAGS_grasp_type == "wide_power" ||
          FLAGS_grasp_type == "basic_power") {
        gripper_x = 0;
        gripper_y = -0.1;
        gripper_z = 0.15;
        gripper_qw = 1;
        gripper_qx = 0;
        gripper_qy = 0;
        gripper_qz = 0;
      }
      else {
        gripper_x = 0;
        gripper_y = -0.15;
        gripper_z = 0.15;
        gripper_qw = 1;
        gripper_qx = 0;
        gripper_qy = 0;
        gripper_qz = 0;
      }
    }
  }

  std::string hand_model_path = FindResourceOrThrow(
    "drake/manipulation/models/"
    "robotiq_3f_description/urdf/robotiq-3f-gripper_articulated_" +
    gripper_mode + ".urdf");

  std::string floor_model_path = FindResourceOrThrow(
    "drake/examples/robotiq_3f/joint_control/floor.urdf");

  std::string slider_model_path = FindResourceOrThrow(
    "drake/examples/robotiq_3f/joint_control/slider.urdf");

  multibody::Parser parser(&plant);
  const multibody::ModelInstanceIndex hand_model_index =
    parser.AddModelFromFile(hand_model_path);
  parser.AddModelFromFile(floor_model_path);
  const multibody::ModelInstanceIndex slider_model_index =
    parser.AddModelFromFile(slider_model_path);

  const std::string object_model_path = FindResourceOrThrow(
    "drake/examples/robotiq_3f/joint_control/" + FLAGS_obj + ".sdf");
  const multibody::ModelInstanceIndex obj_model_index =
    parser.AddModelFromFile(object_model_path);

  // slider_to_hand
  RigidTransformd hand_rigid_tf_orientation(
    Eigen::Quaterniond(gripper_qw, gripper_qx, gripper_qy, gripper_qz),
    Eigen::Vector3d(0, 0, 0));
  RigidTransformd hand_rigid_tf_position(
    RollPitchYawd(0, 0, 0),
    Eigen::Vector3d(gripper_x, gripper_y, gripper_z));

  // Weld the slider to the world frame, with origin at hand's origin position
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("slider_fixed_dummy_link"),
                   hand_rigid_tf_position);

  // Weld the hand to the slider at the slider's origin with the hand's
  // position set
  plant.WeldFrames(plant.GetFrameByName("slider_link"),
                   plant.GetFrameByName("palm"),
                   hand_rigid_tf_orientation);
  RigidTransformd floor_rigid_tf(RollPitchYawd(0, 0, 0),
    Eigen::Vector3d(0, 0, 0));
  // Weld the floor to the world frame
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("floor"),
                   floor_rigid_tf);

  if (FLAGS_remove_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d::Zero());
  }

  // Finished building the plant
  plant.Finalize();

  // TODO(mcorsaro): automate this
  // note: this has to be all links, no matter how many joints are fixed
  std::vector<std::string> link_names_in_order;
  link_names_in_order.push_back("finger_middle_link_1");
  link_names_in_order.push_back("finger_middle_link_2");
  link_names_in_order.push_back("finger_middle_link_3");
  link_names_in_order.push_back("finger_1_link_1");
  link_names_in_order.push_back("finger_1_link_2");
  link_names_in_order.push_back("finger_1_link_3");
  link_names_in_order.push_back("finger_2_link_1");
  link_names_in_order.push_back("finger_2_link_2");
  link_names_in_order.push_back("finger_2_link_3");
  link_names_in_order.push_back("palm");
  // object
  link_names_in_order.push_back("main_body");
  link_names_in_order.push_back("floor");
  std::vector<int> body_indices_in_order(12);

  // get array of body indices to check collision data with
  for (int i=0; i<plant.num_bodies(); i++) {
    multibody::BodyIndex bi(i);
    for (uint j=0; j<link_names_in_order.size(); j++)
    {
      if (plant.get_body(bi).name() == link_names_in_order[j])
      {
        body_indices_in_order[j] = i;
      }
    }
  }

  // Visualization
  geometry::DrakeVisualizer::AddToBuilder(&builder, scene_graph);
  DRAKE_DEMAND(!!plant.get_source_id());

  // Publish contact results for visualization.
  multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant, lcm);

  // ============================== PID Controllers ============================
  // PID controller for position control of the finger joints
  VectorX<double> kp, kd, ki;
  MatrixX<double> Sx, Sy;
  GetControlPortMapping(plant, &Sx, &Sy, num_joints);
  SetPositionControlledGains(&kp, &ki, &kd, num_joints);
  auto& hand_controller = *builder.AddSystem<
      systems::controllers::PidController>(Sx, Sy, kp, ki, kd);
  builder.Connect(plant.get_state_output_port(),
                  hand_controller.get_input_port_estimated_state());

  VectorX<double> kp_slider = Eigen::VectorXd::Constant(1, 5000);
  VectorX<double> kd_slider = Eigen::VectorXd::Constant(1, 1400);
  VectorX<double> ki_slider = Eigen::VectorXd::Constant(1, 0);
  std::vector<drake::multibody::JointIndex> slider_joint_index_mapping;
  slider_joint_index_mapping.push_back(
    plant.GetJointByName("slider_joint").index());
  MatrixX<double> Sx_slider = plant.MakeStateSelectorMatrix(
    slider_joint_index_mapping);
  MatrixX<double> Sy_slider = plant.MakeActuatorSelectorMatrix(
    slider_joint_index_mapping);

  auto& slider_controller =
    *builder.AddSystem<systems::controllers::PidController>(
      Sx_slider, Sy_slider, kp_slider, ki_slider, kd_slider);
  builder.Connect(plant.get_state_output_port(),
                  slider_controller.get_input_port_estimated_state());

  // ============================= Status Converters ===========================
  // Create an output port of the continuous state from the plant that only
  // output the status of the hand finger joints related DOFs, and put them in
  // the pre-defined order that is easy for understanding.
  const auto& hand_status_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sx);
  builder.Connect(plant.get_state_output_port(),
                  hand_status_converter.get_input_port());
  const auto& hand_output_torque_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sy.transpose());
  builder.Connect(hand_controller.get_output_port_control(),
                  hand_output_torque_converter.get_input_port());
  const auto& slider_status_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sx_slider);
  builder.Connect(plant.get_state_output_port(),
                  slider_status_converter.get_input_port());

  // ============================== Controller Out =============================
  // TODO(mcorsaro): don't make this manually
  // This block creates two selector matrices.
  //
  // hand_controller_output_selector is a 9 x 10 that, when multipled by the
  // output of the hand's PID controller with a matrix gain block, outputs the
  // 9 control values in the order expected by the plant's actuation_input_port.
  // In practice, this is a 10x10 Identity with the last row removed.
  // Note that Sy and Sy.transpose() don't work here because they're not
  // identity matrices since the selected order is not the default order.
  //
  // slider_controller_output_selector is a 1 x 10 that, when multipled by the
  // output of the slider's PID controller with a matrix gain block, outputs the
  // 1 control value expected by the plant's actuation_input_port for the
  // slider.
  int pid_out_sz = hand_controller.get_output_port_control().size();
  MatrixX<double> controller_output_selector =
    MatrixX<double>::Identity(pid_out_sz, pid_out_sz);
  MatrixX<double> hand_controller_output_selector =
    controller_output_selector.block(0, 0, pid_out_sz-1, pid_out_sz);
  MatrixX<double> slider_controller_output_selector =
    controller_output_selector.block(pid_out_sz-1, 0, 1, pid_out_sz);

  // =========================== Controller Converters =========================
  const auto& hand_controller_output_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(
        hand_controller_output_selector);
  builder.Connect(hand_controller.get_output_port_control(),
                  hand_controller_output_converter.get_input_port());

  const auto& slider_controller_output_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(
        slider_controller_output_selector);
  builder.Connect(slider_controller.get_output_port_control(),
                  slider_controller_output_converter.get_input_port());

  // ====================== Controller Converters to Plant =====================
  builder.Connect(hand_controller_output_converter.get_output_port(),
                  plant.get_actuation_input_port(hand_model_index));
  builder.Connect(slider_controller_output_converter.get_output_port(),
                  plant.get_actuation_input_port(slider_model_index));

  // =========================== Command Subscribers ===========================
  // Create the command subscriber and status publisher for the hand.
  auto& hand_binary_command_sub = *builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_robotiq_3f_binary_command>(
          "ROBOTIQ3F_BINARY_COMMAND", lcm));
  hand_binary_command_sub.set_name("hand_binary_command_subscriber");
  auto& contact_processor =
      *builder.AddSystem<ContactProcessor>();
  contact_processor.set_name("contact_processor");
  contact_processor.setBodyIDs(body_indices_in_order);

  auto& hand_command_receiver =
      *builder.AddSystem<Robotiq3fCommandReceiver>(num_joints);
  hand_command_receiver.set_name("hand_command_receiver");

  builder.Connect(hand_binary_command_sub.get_output_port(),
                  hand_command_receiver.get_input_port_binary_command());
  builder.Connect(hand_status_converter.get_output_port(),
                  hand_command_receiver.get_input_port_state());

  builder.Connect(plant.get_contact_results_output_port(),
                  contact_processor.get_contact_input_port());
  builder.Connect(contact_processor.get_contact_history_output_port(),
                  hand_command_receiver.get_input_port_contact());

  builder.Connect(hand_command_receiver.get_commanded_state_output_port(),
                  hand_controller.get_input_port_desired_state());

  auto& slider_command_sub = *builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_slider_command>(
          "SLIDER_COMMAND", lcm));
  slider_command_sub.set_name("slider_command_subscriber");
  auto& slider_command_receiver =
      *builder.AddSystem<SliderCommandReceiver>();
  slider_command_receiver.set_name("slider_command_receiver");

  builder.Connect(slider_command_sub.get_output_port(),
                  slider_command_receiver.get_input_port(0));
  builder.Connect(slider_command_receiver.get_commanded_state_output_port(),
                  slider_controller.get_input_port_desired_state());

  // ============================ Status Publishers ============================
  auto& hand_status_pub = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_robotiq_3f_status>(
          "ROBOTIQ3F_STATUS", lcm, kLcmStatusPeriod));
  hand_status_pub.set_name("hand_status_publisher");
  auto& status_sender =
      *builder.AddSystem<Robotiq3fStatusSender>(num_joints);
  status_sender.setBodyIDs(body_indices_in_order);
  status_sender.set_name("status_sender");

  /// Adapted from multibody/plant/test/box_test @98a4cb6
  /// Send contact results to status sender so it can determine if any
  /// links were in contact with object
  builder.Connect(plant.get_contact_results_output_port(),
                  status_sender.get_contact_input_port());

  builder.Connect(hand_status_converter.get_output_port(),
                  status_sender.get_state_input_port());
  builder.Connect(hand_command_receiver.get_output_port(0),
                  status_sender.get_command_input_port());
  builder.Connect(hand_output_torque_converter.get_output_port(),
                  status_sender.get_commanded_torque_input_port());
  builder.Connect(status_sender.get_output_port(0),
                  hand_status_pub.get_input_port());

  auto& obj_status_pub = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_obj_status>(
          "OBJ_STATUS", lcm, kLcmStatusPeriod));
  obj_status_pub.set_name("obj_status_publisher");
  auto& obj_status_sender = *builder.AddSystem<ObjStatusSender>();
  obj_status_sender.set_name("obj_status_sender");
  builder.Connect(plant.get_state_output_port(obj_model_index),
                  obj_status_sender.get_state_input_port());
  builder.Connect(obj_status_sender.get_output_port(0),
                  obj_status_pub.get_input_port());

  auto& slider_status_pub = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_slider_status>(
          "SLIDER_STATUS", lcm, kLcmStatusPeriod));
  slider_status_pub.set_name("slider_status_publisher");
  auto& slider_status_sender = *builder.AddSystem<SliderStatusSender>();
  slider_status_sender.set_name("slider_status_sender");
  builder.Connect(plant.get_state_output_port(slider_model_index),
                  slider_status_sender.get_state_input_port());
  builder.Connect(slider_status_sender.get_output_port(0),
                  slider_status_pub.get_input_port());

  // ================================== Build ==================================

  // Now the model is complete.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Initialize joint angles.
  std::vector<std::string> joint_names_in_order =
    GetPreferredJointOrdering(num_joints);
  // Change initial joint configuration, which was all zeros in the allegro
  // example, so finger_*_link_3's joint is within its limits
  VectorX<double> initial_joint_config =
    VectorX<double>::Zero(num_joints);
  for (uint i=0; i<joint_names_in_order.size(); i++) {
    const multibody::RevoluteJoint<double>& joint =
      plant.GetJointByName<multibody::RevoluteJoint>(joint_names_in_order[i]);
    // Leave at 0 for the distal joint - the last joint on each of the 3 fingers
    if (i%(num_joints/3) != num_joints/3-1) {
      initial_joint_config[i] = joint.position_lower_limit();
    }
    joint.set_angle(&plant_context, initial_joint_config[i]);
  }

  VectorX<double> slider_initial_translation = VectorX<double>::Zero(1);
  const multibody::PrismaticJoint<double>& slider_joint =
      plant.GetJointByName<multibody::PrismaticJoint>("slider_joint");
  slider_joint.set_translation(&plant_context, slider_initial_translation[0]);

  const multibody::Body<double>& obj_body = plant.GetBodyByName("main_body");

  RigidTransformd X_WM(
    RollPitchYawd(0, 0, 0),
    Eigen::Vector3d(0, 0.0, 0.0));
  plant.SetFreeBodyPose(&plant_context, obj_body, X_WM);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  // set the initial command for the hand
  hand_command_receiver.set_initial_position(
      &diagram->GetMutableSubsystemContext(hand_command_receiver,
                                           &simulator.get_mutable_context()),
      initial_joint_config);
  // initialize contact_processor state witin context
  contact_processor.initialize(
      &diagram->GetMutableSubsystemContext(contact_processor,
                                           &simulator.get_mutable_context()));

  // set the initial command for the slider
  slider_command_receiver.set_initial_position(
      &diagram->GetMutableSubsystemContext(slider_command_receiver,
                                           &simulator.get_mutable_context()),
      slider_initial_translation);

  simulator.AdvanceTo(FLAGS_simulation_time);
}

}  // namespace
}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Robotiq3f hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::robotiq_3f::DoMain();
  return 0;
}
