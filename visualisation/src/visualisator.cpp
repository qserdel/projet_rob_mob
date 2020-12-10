#include "ros/ros.h"
#include <cstdlib>
#include <iostream>

#include "nav_msgs/OccupancyGrid.h"
#include "gridmap2d.h"
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"

using namespace gridmap_2d;

static const std::string OPENCV_WINDOW = "Display window";
static GridMap2D gridmap;
static cv::Mat display_map(40,40,0);

void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    std::cout << "map received\n";
    gridmap = GridMap2D(msg);
    display_map = gridmap.binaryMap();
    //cv::imshow(OPENCV_WINDOW, display_map);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visu");
    if (argc != 1)
    {
        ROS_INFO("***ERROR: usage: visualisation with no args");
        return 1;
    }

    ros::NodeHandle n;

    cv::namedWindow(OPENCV_WINDOW);
    ros::Subscriber map_sub = n.subscribe("binary_map", 1000, map_callback);

    //ros::spin();
    while(ros::ok())
    {
        cv::imshow(OPENCV_WINDOW,display_map);
    }
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}