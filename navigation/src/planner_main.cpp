#include "navigation/planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    DriveLimits limits;
    CarRobot car_robot(limits);
    car_robot.displayRobotDetails();

    Planner planner(nh, &car_robot);

    while(ros::ok())
    {
        planner.updateRobotPose();
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    return 0;
}