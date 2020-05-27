/// @file

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
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
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace drake {
namespace examples {
namespace obj_and_floor {
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
DEFINE_string(obj, "cylinder", "object_name - cylinder or box");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
    &builder, FLAGS_max_time_step);
  scene_graph.set_name("scene_graph");

  const std::string floor_model_path = FindResourceOrThrow(
    "drake/examples/obj_and_floor/"
    "models/floor.urdf");

  const std::string object_model_path = FindResourceOrThrow(
    "drake/examples/obj_and_floor/models/" + FLAGS_obj + ".sdf");

  multibody::Parser parser(&plant);
  parser.AddModelFromFile(floor_model_path);
  parser.AddModelFromFile(object_model_path);

  
  RigidTransformd floor_rigid_tf(RollPitchYawd(0, 0, 0),
    Eigen::Vector3d(0, 0, 0));
  // Weld the floor to the world frame
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("floor_link"),
                   floor_rigid_tf);
  
  if (FLAGS_remove_gravity) {
    plant.mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d::Zero());
  }

  // Finished building the plant
  plant.Finalize();

  // Visualization
  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  DRAKE_DEMAND(!!plant.get_source_id());

  // Publish contact results for visualization.
  multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant, lcm);

  // Now the model is complete.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  geometry::DispatchLoadMessage(scene_graph, lcm);
  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  const multibody::Body<double>& obj_body = plant.GetBodyByName("main_body");

  RigidTransformd obj_initial_pose(
    RollPitchYawd(0, 0, 0),
    Eigen::Vector3d(0, 0.0, 0.3));
  plant.SetFreeBodyPose(&plant_context, obj_body, obj_initial_pose);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_simulation_time);
}

}  // namespace
}  // namespace obj_and_floor
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation with an object and floor.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::obj_and_floor::DoMain();
  return 0;
}
