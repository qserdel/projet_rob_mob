#include "ros/ros.h"
#include <cstdlib>
#include <iostream>

#include "nav_msgs/OccupancyGrid.h"
#include "gridmap2d.h"
#include <mapping/BinaryMap.h>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"

using namespace gridmap_2d;

static const std::string OPENCV_WINDOW = "Display window";
static GridMap2D gridmap;
static cv::Mat display_map;

void map_callback(const nav_msgs::OccupancyGrid &msg)
{
    std::cout << "map received\n";
    gridmap = GridMap2D(msg);
    display_map = gridmap.binaryMap();
    //cv::imshow(OPENCV_WINDOW, display_map);
    //cvWaitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visu");
    if (argc != 1)
    {
        ROS_INFO("***ERROR: usage: visualisation with no args");
        return 1;
    }

    ros::NodeHandle n;

    cv::namedWindow(OPENCV_WINDOW);
    //ros::Subscriber map_sub = n.subscribe("binary_map", 5, map_callback);

    ros::ServiceClient map_client = n.serviceClient<mapping::BinaryMap>("binary_map");
    mapping::BinaryMap map_srv;

    ros::Rate loop_rate(0.5);

    //ros::spin();
    while (ros::ok())
    {
        if (map_client.call(map_srv))
        {
            gridmap = GridMap2D(map_srv.response.map);
            display_map = gridmap.binaryMap();
        }
        else{
            ROS_ERROR("Service BinaryMap unavailable");
        }
        if (!display_map.empty())
        {
            cv::imshow(OPENCV_WINDOW, display_map);
            cvWaitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}