#ifndef __EXPLORER_H__
#define __EXPLORER_H__

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

class Explorer
{
    float k1_;
    float k2_;
    float speed_;
    float d0_;

    ros::Subscriber scan_sub_;
    ros::Publisher cmd_pub_;

    //ros::ServiceClient map_client_;

public:
    Explorer();

    void scanCallback(const sensor_msgs::LaserScan& msg);
    void compute_pos(const sensor_msgs::LaserScan& msg, const size_t index, float& x, float& y);
    size_t index_from_angle(float angle, const sensor_msgs::LaserScan& msg);
};

#endif  // __EXPLORER_H__