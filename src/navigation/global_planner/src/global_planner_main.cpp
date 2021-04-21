#include "global_planner/breadth_first_search.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    BFS bfs(&nh);

    ros::Rate rate(1.0);
    while(ros::ok())
    {
        bfs.runGlobalPlanning();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}