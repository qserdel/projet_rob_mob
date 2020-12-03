#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdlib>
#include <iostream>
#include <memory>
#include "gridmap2d.h"
#include <opencv2/highgui/highgui.hpp>

using namespace gridmap_2d;


static const std::string OPENCV_WINDOW = "Image window";

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
    nav_msgs::OccupancyGrid map;

    if (client.call(srv))
    {
        ROS_INFO("Service GetMap succeeded");
        map = srv.response.map;
    }
    else
    {
        ROS_ERROR("Service GetMap failed");
        return 1;
    }

    std::cout << "Map récupérée\n";

    GridMap2D gridmap(map);

    cv::namedWindow(OPENCV_WINDOW);
    imshow(OPENCV_WINDOW, gridmap.binaryMap());
    cvWaitKey(0);


    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}