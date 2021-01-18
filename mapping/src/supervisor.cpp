#include "supervisor.h"

Supervisor::Supervisor() : Mapper()
{
    ros::NodeHandle n("~");

    // Get topic and service names
    std::string map_srv(n.param<std::string>("map_srv", "/dynamic_map"));

    // Initialize subscriber and publisher
    map_client_ = n.serviceClient<nav_msgs::GetMap>(map_srv);
}

Supervisor::~Supervisor() 
{
    
}

void Supervisor::spin() 
{
    std_msgs::Bool done;
    if (!done_ && map_client_.call(map_srv_))
    {
        gridmap_.setMap(map_srv_.response.map, false, 60);
        gridmap_.inflateMap(0.7);
    }
    // end condition
    // TODO

    rate_.sleep();
}