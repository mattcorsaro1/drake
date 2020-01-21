// originally copied from examples/allegro_hand @ f646302

#include "drake/examples/robotiq_3f/robotiq_3f_lcm.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/robotiq_3f/robotiq_3f_common.h"
#include "drake/lcmt_robotiq_3f_command.hpp"
#include "drake/lcmt_robotiq_3f_status.hpp"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace robotiq_3f {

GTEST_TEST(Robotiq3fLcmTest, Robotiq3fCommandReceiver) {
  Robotiq3fCommandReceiver dut;
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput();

  // Check that the commanded pose starts out at zero, and that we can
  // set a different initial position.
  Eigen::VectorXd expected = Eigen::VectorXd::Zero(kRobotiq3fNumJoints * 2);
  dut.CalcOutput(*context, output.get());
  const double tol = 1e-5;
  EXPECT_TRUE(CompareMatrices(
      expected, output->get_vector_data(0)->get_value(),
      tol, MatrixCompareType::absolute));

  Eigen::VectorXd position(kRobotiq3fNumJoints);
  position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.1, 0.1;
  dut.set_initial_position(context.get(), position);
  dut.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(
      position, output->get_vector_data(0)->get_value()
      .head(kRobotiq3fNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      expected.tail(kRobotiq3fNumJoints),
      output->get_vector_data(0)->get_value().tail(kRobotiq3fNumJoints),
      tol, MatrixCompareType::absolute));

  Eigen::VectorXd delta(kRobotiq3fNumJoints);
  delta << 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009,
           0.010, 0.011;

  lcmt_robotiq_3f_command command{};
  command.num_joints = kRobotiq3fNumJoints;
  command.joint_position.resize(kRobotiq3fNumJoints);
  for (int i = 0; i < kRobotiq3fNumJoints; i++) {
    command.joint_position[i] = position(i) + delta(i);
  }

  dut.get_input_port(0).FixValue(context.get(), command);

  std::unique_ptr<systems::DiscreteValues<double>> update =
      dut.AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  dut.CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state().SetFrom(*update);

  dut.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(
      position + delta,
      output->get_vector_data(0)->get_value().head(kRobotiq3fNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      VectorX<double>::Zero(kRobotiq3fNumJoints),
      output->get_vector_data(0)->get_value().tail(kRobotiq3fNumJoints),
      tol, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      VectorX<double>::Zero(kRobotiq3fNumJoints),
      output->get_vector_data(1)->get_value(),
      tol, MatrixCompareType::absolute));
}

GTEST_TEST(Robotiq3fLcmTest, Robotiq3fStatusSenderTest) {
  Robotiq3fStatusSender dut;
  std::unique_ptr<systems::Context<double>>
      context = dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput();

  Eigen::VectorXd position(kRobotiq3fNumJoints);
  position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.1, 0.1;

  Eigen::VectorXd command = Eigen::VectorXd::Zero(kRobotiq3fNumJoints * 2);
  command.head(kRobotiq3fNumJoints) = position * 0.5;
  dut.get_command_input_port().FixValue(context.get(), command);

  Eigen::VectorXd state = Eigen::VectorXd::Zero(kRobotiq3fNumJoints * 2);
  state.head(kRobotiq3fNumJoints) = position;
  state.tail(kRobotiq3fNumJoints) << 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2,
                                   0.1, -0.04, -0.04;
  dut.get_state_input_port().FixValue(context.get(), state);

  Eigen::VectorXd torque = Eigen::VectorXd::Zero(kRobotiq3fNumJoints);
  torque << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1;;
  dut.get_commanded_torque_input_port().FixValue(context.get(), torque);

  dut.CalcOutput(*context, output.get());
  lcmt_robotiq_3f_status status =
      output->get_data(0)->get_value<lcmt_robotiq_3f_status>();
  ASSERT_EQ(status.num_joints, kRobotiq3fNumJoints);
  for (int i = 0; i < kRobotiq3fNumJoints; i++) {
    EXPECT_EQ(status.joint_position_commanded[i], command(i));
    EXPECT_EQ(status.joint_position_measured[i], state(i));
    EXPECT_EQ(status.joint_velocity_estimated[i], state(
                                                  i + kRobotiq3fNumJoints));
    EXPECT_EQ(status.joint_torque_commanded[i], torque(i));
  }
}

}  // namespace robotiq_3f
}  // namespace examples
}  // namespace drake
