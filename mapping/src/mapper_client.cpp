#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdlib>
#include <iostream>
#include <memory>
#include "gridmap2d.h"
#include <mapping/BinaryMap.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include "opencv2/highgui.hpp"

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
        //std::cout << "je suis là\n";
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

    // OpenCV treatment
    /*cv::Mat map_dilate;
    int erosion_size = 3;
    float dilation_size = 3.5;
    int erosion_size2 = 20;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT,
                                                 cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                 cv::Point(dilation_size, dilation_size));

    cv::Mat element3 = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                 cv::Size(2 * erosion_size2 + 1, 2 * erosion_size2 + 1),
                                                 cv::Point(erosion_size2, erosion_size2));*/

    // case of a static map
    if (client.call(srv))
    {
        ROS_INFO("Service GetMap succeeded");
        binary_map = srv.response.map;
        gridmap = GridMap2D(binary_map, false, 60);

        /*cv::erode(gridmap.binaryMap(), map_dilate, element);
        cv::dilate(map_dilate, map_dilate, element2);
        cv::erode(map_dilate, map_dilate, element3);*/
        gridmap.inflateMap(0.6);

        //gridmap.setMap(map_dilate);
        binary_map = gridmap.toOccupancyGridMsg();
    }
    else
    {
        ROS_ERROR("Service GetMap failed");
        return 1;
    }

    ros::spin();

    return 0;
}