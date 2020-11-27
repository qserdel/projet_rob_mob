#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapper_client");
    if (argc != 1)
    {
        ROS_INFO("***ERROR: usage: mapper_client with no args");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;
    
    if (client.call(srv))
    {
        ROS_INFO("Service GetMap succeeded");
    }
    else
    {
        ROS_ERROR("Service GetMap failed");
        return 1;
    }
    nav_msgs::OccupancyGrid map = srv.response.map;
    std::cout << "Map récupérée\n";
    return 0;
}