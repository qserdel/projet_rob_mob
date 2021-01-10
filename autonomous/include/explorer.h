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
    /**
     * @brief Construct a new Explorer object
     * 
     */
    Explorer();

    /**
     * @brief Callback function on Lidar measure topic.
     * 
     * @param msg The Lidar measure topic.
     */
    void scanCallback(const sensor_msgs::LaserScan &msg);

    /**
     * @brief Get the coordinate of point measured at the index 
     * in the Lidar measure topic.
     * 
     * @param msg The lidar measure topic.
     * @param index The index of the point we want to know its coordinate.
     * @param x The x coordinate of the point.
     * @param y The y coordinate of the point.
     */
    void compute_pos(const sensor_msgs::LaserScan &msg, const size_t index, float &x, float &y);

    /**
     * @brief Get the correct index in the list of measure of the Lidar topic.
     *  
     * @param angle The angle we want to know the index of.
     * @param msg The Lidar measure topic.
     * @return size_t The index of the list corresponding to the desired angle.
     */
    size_t index_from_angle(float angle, const sensor_msgs::LaserScan &msg);
};

#endif // __EXPLORER_H__