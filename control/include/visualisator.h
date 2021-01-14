#ifndef __VISUALISATOR_H__
#define __VISUALISATOR_H__

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "gridmap2d.h"
#include "autonomous/BinaryMap.h"
#include "control/Traj.h"

struct Robot
{
    cv::Point pos;
    unsigned int radius;
    cv::Vec3b color;
};

struct Tree
{
    std::vector<cv::Point> nodes;
    cv::Vec3b color;
    unsigned int thickness;
};

class Visualisator
{
    gridmap_2d::GridMap2D gridmap_;
    cv:Mat display_map;
    Robot robot_;
    Tree tree_;

    ros::ServiceClient map_client_; ///< Client du service /binary_map
    autonomous::BinryMap map_srv_;

    ros::ServiceClient traj_client_; ///< Client du service /checkpoints
    control::Traj traj_srv_;

    ros::Subsciber odom_sub_;

};
#endif // __VISUALISATOR_H__