#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdlib>
#include <iostream>
#include <memory>
#include "gridmap2d.h"
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"

using namespace gridmap_2d;

//static const std::string OPENCV_WINDOW = "Image window";

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
    nav_msgs::OccupancyGrid map;
    GridMap2D gridmap;

    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("binary_map", 1000);

    cv::Mat map_dilate;
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
                                                 cv::Point(erosion_size2, erosion_size2));

    if (client.call(srv))
    {
        ROS_INFO("Service GetMap succeeded");
        map = srv.response.map;
        gridmap = GridMap2D(map, false, 60);

        cv::erode(gridmap.binaryMap(), map_dilate, element);
        cv::dilate(map_dilate, map_dilate, element2);
        cv::erode(map_dilate, map_dilate, element3);

        gridmap.setMap(map_dilate);
        map = gridmap.toOccupancyGridMsg();
        map_pub.publish(map);
        std::cout << "map published\n";
    }
    else
    {
        ROS_ERROR("Service GetMap failed");
        return 1;
    }

    /*std::cout << "Map récupérée\n";

    cv::namedWindow(OPENCV_WINDOW);
    imshow(OPENCV_WINDOW, gridmap.binaryMap());
    cvWaitKey(0);

    cv::destroyWindow(OPENCV_WINDOW);*/
    ros::spin();

    return 0;
}