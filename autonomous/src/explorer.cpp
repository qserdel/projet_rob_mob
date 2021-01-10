#include "explorer.h"

#include <string>
#include <iostream>
#include <cmath>

#include "ros/console.h"
#include "geometry_msgs/Twist.h"

Explorer::Explorer() : k1_(0.0f), k2_(0.0f), speed_(0.0f), d0_(0.0f), scan_sub_(), cmd_pub_()
{
    ros::NodeHandle n("~");
    speed_ = n.param("speed", 1.0f);
    k1_ = n.param("k1", 1.0f);
    k2_ = n.param("k2", 1.0f);
    d0_ = n.param("distance", 1.0f);
    std::string scan_topic(n.param<std::string>("scan_topic", "/scan"));
    std::string cmd_topic(n.param<std::string>("cmd_topic", "/cmd_vel"));

    // Initialize subscriber and publisher
    scan_sub_ = n.subscribe(scan_topic, 1, &Explorer::scanCallback, this);
    cmd_pub_ = n.advertise<geometry_msgs::Twist>(cmd_topic, 1);
}

void Explorer::scanCallback(const sensor_msgs::LaserScan &msg)
{
    size_t i_min(0);
    size_t ranges_len(msg.ranges.size());

    //std::cout << "\n-------\n";
    //std::cout << speed_ << "\t" << k1_ << "\t" << k2_ << "\t" << d0_ << std::endl;
    ROS_INFO("speed: %f\tk1: %f\tk2: %f\td0: %f", speed_, k1_, k2_, d0_);
    //std::cout << "[" << msg.angle_min << ":" << msg.angle_increment << ":" << msg.angle_max << "]" << std::endl;

    // Find the index of the closest obstacle
    for (size_t i = 1; i < ranges_len; i++)
    {
        if (msg.ranges.at(i) < msg.ranges.at(i_min))
            i_min = i;
    }

    size_t index_i, index_j;
    index_i = 0;
    index_j = index_from_angle(-M_PI / 4, msg);
    //std::cout << "ind_i: " << index_i << "\tind_j: " << index_j << std::endl;

    // Get points of the obstacle to dertermine its direction
    float xi(0.0f), yi(0.0f), xj(0.0f), yj(0.0f);
    compute_pos(msg, index_i, xi, yi);
    compute_pos(msg, index_j, xj, yj);

    std::cout << "i: [" << xi << ":" << yi << "]\n";
    std::cout << "j: [" << xj << ":" << yj << "]\n";

    geometry_msgs::Twist cmd;
    cmd.linear.x = speed_;

    float dist = msg.ranges.at(i_min);
    std::cout << "dist: " << dist << std::endl;
    float tan((yi - yj) / (xi - xj));
    float angle(std::atan2(yi - yj, xi - xj));
    std::cout << "angle: " << angle << std::endl;
    cmd.angular.z = k1_ * (dist - d0_) / (speed_ * std::cos(angle)) - k2_ * -tan;
    cmd_pub_.publish(cmd);

    //std::cout << cmd << std::endl;
}

void Explorer::compute_pos(const sensor_msgs::LaserScan &msg, const size_t index, float &x, float &y)
{
    float angle(msg.angle_min + msg.angle_increment * index);
    float dist(msg.ranges.at(index));
    x = dist * std::cos(angle);
    y = dist * std::sin(angle);
}

size_t Explorer::index_from_angle(float angle, const sensor_msgs::LaserScan &msg)
{
    return (angle - msg.angle_min) / msg.angle_increment;
}