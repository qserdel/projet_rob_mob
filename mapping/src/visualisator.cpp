#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "gridmap2d.h"
#include <mapping/BinaryMap.h>
#include <planification/Checkpoints.h>
#include <planification/ListePoints.h>

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

struct CheckpointsTraj
{
    std::vector<cv::Point> points;
    unsigned int len;
    cv::Vec3b color;
    unsigned int thickness;
};
CheckpointsTraj traj;

void setCheckpointsTraj(CheckpointsTraj &cpt, const planification::Checkpoints &cp_list, cv::Vec3b col, unsigned int thickness)
{
    cpt.points.clear();
    cpt.len = 0;
    cpt.color = col;
    cpt.thickness = thickness;
    unsigned int mx, my;

    for (const geometry_msgs::Point &pt : cp_list.response.points.points)
    {
        if (gridmap.inMapBounds(pt.x, pt.y))
        {
            gridmap.worldToMap(pt.x, pt.y, mx, my);
            cpt.points.push_back(cv::Point(my, mx));
            cpt.len++;
        }
    }
}

void showTraj(const CheckpointsTraj &cpt, cv::Mat& dest)
{
    for (int i=1 ; i<cpt.len ; i++)
    {
        cv::line(dest, cpt.points[i-1], cpt.points[i], cpt.color, cpt.thickness);
    }
}

void printCheckPointsTraj(const CheckpointsTraj &cpt)
{
    std::cout << "Trajectoire de checkpoints:\n";
    for (const cv::Point &pt : cpt.points)
    {
        std::cout << pt << ",";
    }
    std::cout << "\n";
}

void odomCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::Pose robot_pos = msg.pose.pose;
    unsigned int mx, my;
    if (gridmap.inMapBounds(robot_pos.position.x, robot_pos.position.y))
    {
        gridmap.worldToMap(robot_pos.position.x, robot_pos.position.y, mx, my);
        robot.pos.x = my;
        robot.pos.y = mx;
    }
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
    ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);
    robot.pos = cv::Point();
    robot.radius = 10;
    robot.color = cv::Vec3b(200, 0, 0);

    // Client du service /checkpoints
    ros::ServiceClient checkpoint_client = n.serviceClient<planification::Checkpoints>("checkpoints");
    planification::Checkpoints cp_srv;
    cv::Vec3b traj_col(0,0,200);

    ros::Rate loop_rate(30);
    ros::Rate map_rate(1);

    // OpenCV window
    cv::namedWindow(OPENCV_WINDOW);

    // Wait to receive the map
    while (!map_client.call(map_srv))
    {
        ros::spinOnce();
        map_rate.sleep();
    }
    // Loading of the binary map
    gridmap = GridMap2D(map_srv.response.map);
    //display_map = gridmap.binaryMap();
    cv::cvtColor(gridmap.binaryMap(), display_map, cv::COLOR_GRAY2RGB);

    ros::Time time = ros::Time::now();
    ros::Duration delay_traj(10.);

    // continuous display
    while (ros::ok())
    {
        if (!display_map.empty())
        {
            display_map = cv::Mat::zeros(display_map.rows, display_map.cols, CV_8U);
            cv::cvtColor(gridmap.binaryMap(), display_map, cv::COLOR_GRAY2RGB);

            // Affichage du robot dans la map
            if (robot.pos.x != 0 && robot.pos.y != 0)
            {
                cv::circle(display_map, robot.pos, robot.radius, robot.color, CV_FILLED);
            }

            // Récupération de la liste de checkpoint
            if (ros::Time::now()-time > delay_traj && checkpoint_client.call(cp_srv))
            {
                setCheckpointsTraj(traj, cp_srv, traj_col, 1);
                printCheckPointsTraj(traj);
                time = ros::Time::now();
            }

            // Affichage de la liste de checkpoints
            if (traj.len > 0)
            {
                showTraj(traj, display_map);
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