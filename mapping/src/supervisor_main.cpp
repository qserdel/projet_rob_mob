#include "ros/ros.h"
#include "supervisor.h"

#include <memory>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    ros::NodeHandle n;
    std::shared_ptr<Mapper> supervisor(new Supervisor());

    while (ros::ok())
    {
        supervisor->spin();
        ros::spinOnce();
    }
    return 0;
}