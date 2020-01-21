/// @file
///
/// This file adds a robotiq_3f hand and an object to a simulation environment.
/// The system is designed for position control of the hand, with a PID
/// controller to control the output torque. The system communicate with the
/// external program through LCM system, with a publisher to publish the
/// current state of the hand, and a subscriber to read the posiiton commands
/// of the finger joints.
///
/// originally copied from examples/allegro_hand @ f646302

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/robotiq_3f/robotiq_3f_common.h"
#include "drake/examples/robotiq_3f/robotiq_3f_lcm.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcmt_robotiq_3f_command.hpp"
#include "drake/lcmt_robotiq_3f_status.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
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

namespace drake {
namespace examples {
namespace robotiq_3f {
namespace {

using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::MultibodyPlant;

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds");
DEFINE_bool(use_right_hand, true,
            "Which hand to model: true for right hand or false for left hand");
DEFINE_double(max_time_step, 1.5e-4,
              "Simulation time step used for intergrator.");
DEFINE_bool(add_gravity, false,
            "Whether adding gravity (9.81 m/s^2) in the simulation");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  std::string hand_model_path = FindResourceOrThrow(
        "drake/manipulation/models/"
        "robotiq_3f_description/sdf/robotiq-3f-gripper_articulated.sdf");

  const std::string object_model_path = FindResourceOrThrow(
      "drake/examples/robotiq_3f/joint_control/simple_mug.sdf");
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(hand_model_path);
  parser.AddModelFromFile(object_model_path);

  // Create a transform representing the desired initial hand rotation
  Eigen::Matrix3d hand_pose;
  hand_pose << 0, -1, 0,
               0, 0, -1,
               1, 0, 0;
  math::RotationMatrix hand_pose_rotmat(hand_pose);
  RigidTransformd hand_rigid_tf(hand_pose_rotmat);

  // Weld the hand to the world frame
  const auto& joint_palm = plant.GetBodyByName("palm");
  plant.AddJoint<multibody::WeldJoint>("weld_hand", plant.world_body(),
                                       std::nullopt,
                                       joint_palm,
                                       std::nullopt,
                                       hand_rigid_tf);

  if (!FLAGS_add_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d::Zero());
  }

  // Finished building the plant
  plant.Finalize();

  // Visualization
  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  // Publish contact results for visualization.
  multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant, lcm);

  // PID controller for position control of the finger joints
  VectorX<double> kp, kd, ki;
  MatrixX<double> Sx, Sy;
  GetControlPortMapping(plant, &Sx, &Sy);
  SetPositionControlledGains(&kp, &ki, &kd);
  auto& hand_controller = *builder.AddSystem<
      systems::controllers::PidController>(Sx, Sy, kp, ki, kd);
  builder.Connect(plant.get_state_output_port(),
                  hand_controller.get_input_port_estimated_state());
  builder.Connect(hand_controller.get_output_port_control(),
                  plant.get_actuation_input_port());

  // Create an output port of the continuous state from the plant that only
  // output the status of the hand finger joints related DOFs, and put them in
  // the pre-defined order that is easy for understanding.
  const auto& hand_status_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sx);
  builder.Connect(plant.get_state_output_port(),
                  hand_status_converter.get_input_port());
  const auto& hand_output_torque_converter =
      *builder.AddSystem<systems::MatrixGain<double>>(Sy);
  builder.Connect(hand_controller.get_output_port_control(),
                  hand_output_torque_converter.get_input_port());

  // Create the command subscriber and status publisher for the hand.
  auto& hand_command_sub = *builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_robotiq_3f_command>(
          "ROBOTIQ3F_COMMAND", lcm));
  hand_command_sub.set_name("hand_command_subscriber");
  auto& hand_command_receiver =
      *builder.AddSystem<Robotiq3fCommandReceiver>(kRobotiq3fNumJoints);
  hand_command_receiver.set_name("hand_command_receiver");
  auto& hand_status_pub = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_robotiq_3f_status>(
          "ROBOTIQ3F_STATUS", lcm, kLcmStatusPeriod /* publish period */));
  hand_status_pub.set_name("hand_status_publisher");
  auto& status_sender =
      *builder.AddSystem<Robotiq3fStatusSender>(kRobotiq3fNumJoints);
  status_sender.set_name("status_sender");

  builder.Connect(hand_command_sub.get_output_port(),
                  hand_command_receiver.get_input_port(0));
  builder.Connect(hand_command_receiver.get_commanded_state_output_port(),
                  hand_controller.get_input_port_desired_state());
  builder.Connect(hand_status_converter.get_output_port(),
                  status_sender.get_state_input_port());
  builder.Connect(hand_command_receiver.get_output_port(0),
                  status_sender.get_command_input_port());
  builder.Connect(hand_output_torque_converter.get_output_port(),
                  status_sender.get_commanded_torque_input_port());
  builder.Connect(status_sender.get_output_port(0),
                  hand_status_pub.get_input_port());

  // Now the model is complete.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  geometry::DispatchLoadMessage(scene_graph, lcm);
  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  // Set the position of object
  const multibody::Body<double>& mug = plant.GetBodyByName("main_body");
  const multibody::Body<double>& hand = plant.GetBodyByName("palm");
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Initialize the mug pose to be right in the middle between the fingers.
  const Eigen::Vector3d& p_WHand =
      plant.EvalBodyPoseInWorld(plant_context, hand).translation();
  RigidTransformd X_WM(
      RollPitchYawd(M_PI / 2, 0, 0),
      // For now, send the mug above the hand so we can resolve controller
      // issues.
      p_WHand + Eigen::Vector3d(0.095, 0.062, 0.095+1.0));
  plant.SetFreeBodyPose(&plant_context, mug, X_WM);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  // Change initial joint configuration, which was all zeros in the allegro
  // example, so finger_*_link_3's joint is within its limits
  VectorX<double> initial_joint_config =
    VectorX<double>::Zero(plant.num_actuators());
  initial_joint_config(2) = -0.053;
  initial_joint_config(5) = -0.053;
  initial_joint_config(8) = -0.053;

  // set the initial command for the hand
  hand_command_receiver.set_initial_position(
      &diagram->GetMutableSubsystemContext(hand_command_receiver,
                                           &simulator.get_mutable_context()),
      initial_joint_config);

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
