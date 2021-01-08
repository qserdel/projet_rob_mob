#include <cstdlib>
#include <iostream>
#include <memory>

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "gridmap2d.h"
#include "explorer.h"

using namespace gridmap_2d;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_robot");
    /*if (argc != 1)
    {
        ROS_INFO("***ERROR: usage: mapper with no args");
        return 1;
    }

    ros::NodeHandle n;

    ros::ServiceClient map_client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
    nav_msgs::GetMap map_srv;
    nav_msgs::OccupancyGrid the_map;
    gridmap_2d::GridMap2D gridmap();

    ros::Rate map_rate(0.2);

    while(ros::ok())
    {
        if(map_client.call(map_srv))
        {
            gridmap = GridMap2D(map_srv.response.map, false, 60);
            gridmap.inflateMap(0.5);
        }

        ros::spinOnce();
        map_rate.sleep();
    }*/
    ros::NodeHandle n;
    Explorer X;
    ros::spin();
    return 0;

}