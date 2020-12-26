#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdlib>
#include <iostream>
#include <memory>
#include "gridmap2d.h"
#include <mapping/BinaryMap.h>

using namespace gridmap_2d;

static nav_msgs::OccupancyGrid binary_map;
static GridMap2D gridmap;

/**
 * Service server function delivering the binary map dilated.
 * 
 * @param res the request of the BinaryMap service: none
 * @param req the response of the BinaryMap service: OccupancyGrid
 * @return Wether or not the response is set correctly.
 */
bool publish(mapping::BinaryMap::Request &req, mapping::BinaryMap::Response &res)
{
    if (!gridmap.binaryMap().empty())
    {
        res.map = binary_map;
        return true;
    }
    else return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapper");
    if (argc != 1)
    {
        ROS_INFO("***ERROR: usage: mapper with no args");
        return 1;
    }

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;

    ros::ServiceServer map_srv = n.advertiseService("binary_map", publish);
    ros::Rate static_map_rate(1);

    // case of a static map
    while (!client.call(srv))
    {
        ros::spinOnce();
        static_map_rate.sleep();
    }
    ROS_INFO("Service GetMap succeeded");
    binary_map = srv.response.map;
    gridmap = GridMap2D(binary_map, false, 60);
    gridmap.inflateMap(0.7);
    binary_map = gridmap.toOccupancyGridMsg();

    ros::spin();

    return 0;
}