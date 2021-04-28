#include <ros/ros.h>
#include <gtest/gtest.h>
#include "local_planner/dynamic_window_approach.h"

#define NEAR_MARGIN 1e-7

TEST(DWALocalPlanner, robotInitStateTest)
{
  ros::NodeHandle nh;
  DWA dwa(&nh, "dwa_local_plan");
  State init_state = dwa.robot_->getState();
  ASSERT_EQ(init_state.x, 0.0);
  ASSERT_EQ(init_state.y, 0.0);
  ASSERT_NEAR(init_state.theta, M_PI / 2, NEAR_MARGIN);
  ASSERT_EQ(init_state.v, 0.0);
  ASSERT_EQ(init_state.w, 0.0);
}

TEST(DWALocalPlanner, statePropagationTest)
{
  ros::NodeHandle nh;
  DWA dwa(&nh, "dwa_local_plan");
  dwa.robot_->updateRobotState(0.1, 0.0, 0.05);

  State state = dwa.robot_->getState();

  ASSERT_NEAR(state.x, 0.0, NEAR_MARGIN);
  ASSERT_NEAR(state.y, 0.005, NEAR_MARGIN);
  ASSERT_NEAR(state.theta, M_PI / 2, NEAR_MARGIN);
  ASSERT_NEAR(state.v, 0.1, NEAR_MARGIN);
  ASSERT_NEAR(state.w, 0.0, NEAR_MARGIN);
}

TEST(DWALocalPlanner, generateVelocitySamplesTest)
{
	ros::NodeHandle nh;
  DWA dwa(&nh, "dwa_local_plan");
  
	
	
	dwa.robot_->updateRobotState(0.1, 0.0, 0.05);

  
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dwa_tests");
  return RUN_ALL_TESTS();
}