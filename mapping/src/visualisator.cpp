#include "ros/ros.h"
#include <cstdlib>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"

#include "nav_msgs/OccupancyGrid.h"
#include "gridmap2d.h"
#include <mapping/BinaryMap.h>

using namespace gridmap_2d;

static const std::string OPENCV_WINDOW = "Display window";
static GridMap2D gridmap;
static cv::Mat display_map;

/*void map_callback(const nav_msgs::OccupancyGrid &msg)
{
    ROS_INFO("Map received");

    // récupération de la map
    gridmap = GridMap2D(msg);

    // récupération de la matrice openCV binarisée et dilatée
    display_map = gridmap.binaryMap();
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visu");
    if (argc != 1)
    {
        ROS_ERROR("***ERROR: usage: visualisation with no args");
        return 1;
    }

    ros::NodeHandle n;


    ros::ServiceClient map_client = n.serviceClient<mapping::BinaryMap>("binary_map");
    mapping::BinaryMap map_srv;

    
    ros::Rate loop_rate(1);
    
    cv::namedWindow(OPENCV_WINDOW);
    while (ros::ok())
    {
        if (map_client.call(map_srv))
        {
            gridmap = GridMap2D(map_srv.response.map);
            display_map = gridmap.binaryMap();
        }
        else{
            ROS_INFO("Service BinaryMap unavailable");
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