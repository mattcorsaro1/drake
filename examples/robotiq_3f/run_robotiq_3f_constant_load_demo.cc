/// @file
///
/// This demo sets up a simple dynamic simulation for the Robotiq3f hand using
/// the multi-body library. A single, constant torque is applied to all joints
/// and defined by a command-line parameter.
///
/// originally copied from examples/allegro_hand @ f646302

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

using drake::multibody::MultibodyPlant;

DEFINE_double(constant_load, 0, "the constant load on each joint, Unit [Nm]."
              "Suggested load is in the order of 0.01 Nm. When input value"
              "equals to 0 (default), the program runs a passive simulation.");

DEFINE_double(simulation_time, 5,
              "Desired duration of the simulation in seconds");

DEFINE_bool(use_right_hand, true,
            "Which hand to model: true for right hand or false for left hand");

DEFINE_double(max_time_step, 1.0e-4,
              "Simulation time step used for integrator.");

DEFINE_bool(add_gravity, true, "Indicator for whether terrestrial gravity"
                                " (9.81 m/s²) is included or not.");

DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>
                                  (FLAGS_max_time_step);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  std::string full_name = FindResourceOrThrow("drake/manipulation/models/"
        "robotiq_3f_description/urdf/robotiq-3f-gripper_articulated.urdf");

  multibody::Parser(&plant).AddModelFromFile(full_name);

  // Frame that points hand upwards s.t. the approach direction is parallel to
  // the z-axis.
  Eigen::Matrix3d hand_pose;
  hand_pose << 0, 0, -1,
              -1, 0, 0,
               0, 1, 0;
  math::RotationMatrix hand_pose_rotmat(hand_pose);
  math::RigidTransformd hand_rigid_tf(hand_pose_rotmat);

  // Weld the hand to the world frame
  const auto& joint_palm = plant.GetBodyByName("palm");
  plant.AddJoint<multibody::WeldJoint>("weld_hand", plant.world_body(),
      std::nullopt, joint_palm, std::nullopt,
      hand_rigid_tf);

  if (!FLAGS_add_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d::Zero());
  }

  // Now the model is complete.
  plant.Finalize();

  DRAKE_DEMAND(plant.num_actuators() == 9);
  DRAKE_DEMAND(plant.num_actuated_dofs() == 9);

  // constant force input
  VectorX<double> constant_load_value = VectorX<double>::Ones(
      plant.num_actuators()) * FLAGS_constant_load;
  auto constant_source =
     builder.AddSystem<systems::ConstantVectorSource<double>>(
      constant_load_value);
  constant_source->set_name("constant_source");
  builder.Connect(constant_source->get_output_port(),
                  plant.get_actuation_input_port());

  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Initialize joint angle. 3 joints on the middle and other two fingers
  // are set to some arbitrary values.
  const multibody::RevoluteJoint<double>& joint_finger_1_middle =
      plant.GetJointByName<multibody::RevoluteJoint>("finger_1_joint_2");
  joint_finger_1_middle.set_angle(&plant_context, 0.6);
  const multibody::RevoluteJoint<double>& joint_finger_2_root =
      plant.GetJointByName<multibody::RevoluteJoint>("finger_2_joint_1");
  joint_finger_2_root.set_angle(&plant_context, 0.75);
  const multibody::RevoluteJoint<double>& joint_finger_middle_tip =
      plant.GetJointByName<multibody::RevoluteJoint>("finger_middle_joint_3");
  joint_finger_middle_tip.set_angle(&plant_context, -0.2);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
}

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
