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
#include "trajectory.h"

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

struct Tree
{
    std::vector<cv::Point> nodes;
    cv::Vec3b color;
    unsigned int thickness;
};
static Tree tree;

void buildTree(Tree &tree, cv::Vec3b col, unsigned int thickness)
{
    tree.color = col;
    tree.thickness = thickness;
}

void displayTree(cv::Mat &dest, Tree &tree)
{
    for (size_t i = 1; i < tree.nodes.size(); i += 2)
    {
        cv::line(dest, tree.nodes.at(i - 1), tree.nodes.at(i), tree.color, tree.thickness);
    }
}

void treeCallback(const planification::ListePoints &msg)
{
    unsigned int mx, my;
    for (const geometry_msgs::Point &pt : msg.points)
    {
        std::cout<<pt.x<<","<<pt.y<<std::endl;
        tree.nodes.push_back(cv::Point(pt.x, pt.y));
        /*if (gridmap.inMapBounds(pt.x, pt.y))
        {
            gridmap.worldToMap(pt.x, pt.y, mx, my);
            tree.nodes.push_back(cv::Point(my, mx));

        }
        else
        {
            std::cout << "***WARNING:\n" << pt << " not in bound!\n";
        }*/
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
    ros::Subscriber odom_sub = n.subscribe("odom", 5, odomCallback);
    robot.pos = cv::Point();
    robot.radius = 10;
    robot.color = cv::Vec3b(200, 0, 0);

    // Client du service /checkpoints
    ros::ServiceClient checkpoint_client = n.serviceClient<planification::Checkpoints>("checkpoints");
    planification::Checkpoints cp_srv;
    Trajectory traj;

    ros::Subscriber tree_sub = n.subscribe("segments_rrt", 10, treeCallback);
    buildTree(tree, cv::Vec3b(100, 100, 100), 1);

    ros::Rate loop_rate(30);
    ros::Rate map_rate(1);

    // OpenCV window
    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_NORMAL);
    cv::resizeWindow(OPENCV_WINDOW,500,500);

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
            if (ros::Time::now() - time > delay_traj && checkpoint_client.call(cp_srv))
            {
                traj.setTrajectory(cp_srv, gridmap);
                traj.print();
                time = ros::Time::now();
            }

            // affichage du rrt
            if (tree.nodes.size() > 0)
            {
                displayTree(display_map, tree);
            }

            // Affichage de la liste de checkpoints
            if (traj.size() > 0)
            {
                traj.displayTraj(display_map);
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
