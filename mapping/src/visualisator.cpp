#include "ros/ros.h"
#include <cstdlib>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

#include "gridmap2d.h"
#include <mapping/BinaryMap.h>

using namespace gridmap_2d;

// OpenCV window
static const std::string OPENCV_WINDOW = "Display window";

// Map variables
static GridMap2D gridmap;
static cv::Mat display_map;

// Odom variables
struct Robot
{
    cv::Point pos;
    int radius;
    cv::Vec3b color;
};
static Robot robot;

/*void map_callback(const nav_msgs::OccupancyGrid &msg)
{
    ROS_INFO("Map received");

    // récupération de la map
    gridmap = GridMap2D(msg);

    // récupération de la matrice openCV binarisée et dilatée
    display_map = gridmap.binaryMap();
}*/

void odomCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::Pose robot_pos = msg.pose.pose;
    robot.pos.x = robot_pos.position.x;
    robot.pos.y = robot_pos.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visu");
    if (argc != 1)
    {
        ROS_ERROR("***ERROR: usage: visualisation with no args");
        return 1;
    }

    ros::NodeHandle n;

    // Client du service /binary_map
    ros::ServiceClient map_client = n.serviceClient<mapping::BinaryMap>("binary_map");
    mapping::BinaryMap map_srv;

    // Subscriber du topic /odom
    ros::Subscriber odom_sub = n.subscribe("odom", 30, odomCallback);
    robot.pos = cv::Point();
    robot.radius = 5;
    robot.color = cv::Vec3b(100, 0, 0);

    ros::Rate loop_rate(30);
    ros::Rate map_rate(1);

    // OpenCV winddow
    cv::namedWindow(OPENCV_WINDOW);

    // Wait to receive the map
    while (!map_client.call(map_srv))
    {
        ros::spinOnce();
        map_rate.sleep();
    }
    // Loading of the binary map
    gridmap = GridMap2D(map_srv.response.map);
    display_map = gridmap.binaryMap();

    // continuous display
    while (ros::ok())
    {
        if (!display_map.empty())
        {
            if (robot.pos.x != 0 && robot.pos.y != 0)
            {
                cv::circle(display_map, robot.pos, robot.radius, robot.color);
            }
            cv::imshow(OPENCV_WINDOW, display_map);
            cvWaitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyWindow(OPENCV_WINDOW);

    return 0;
}