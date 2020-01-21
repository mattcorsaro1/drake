// This test is based on
// manipulation/models/wsg_50_description/urdf/test/wsg50_mesh_collision_test.cc
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that robotiq-3f-gripper_articulated.urdf can be parsed into a plant.
GTEST_TEST(Robotiq3fDescriptionTest, TestMeshCollisionModelLoadTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/robotiq_3f_description/urdf/"
      "robotiq-3f-gripper_articulated.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);
  plant.Finalize();

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is a model instance for the gripper.
  EXPECT_EQ(plant.num_model_instances(), 3);

  // following logic from
  // manipulation/models/allegro_hand_description/test/parse_test.cc:
  // 11 for finger joints, 7 for the free moving hand in the space
  EXPECT_EQ(plant.num_positions(), 18);
  EXPECT_EQ(plant.num_velocities(), 17);

  ASSERT_EQ(plant.num_bodies(), 14);
  const std::vector<std::string> expected_body_names {
      "" /* Ignore the world body name */,
      "finger_1_link_0",
      "finger_1_link_1",
      "finger_1_link_2",
      "finger_1_link_3",
      "finger_2_link_0",
      "finger_2_link_1",
      "finger_2_link_2",
      "finger_2_link_3",
      "finger_middle_link_0",
      "finger_middle_link_1",
      "finger_middle_link_2",
      "finger_middle_link_3",
      "palm"};
  for (multibody::BodyIndex i{1}; i < plant.num_bodies(); ++i) {
    EXPECT_THAT(plant.get_body(i).name(),
                testing::EndsWith(expected_body_names.at(i)));
  }
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
