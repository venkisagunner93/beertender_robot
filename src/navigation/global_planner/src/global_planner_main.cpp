#include "global_planner/global_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}