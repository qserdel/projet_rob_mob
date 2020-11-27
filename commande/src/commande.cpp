# include "ros/ros.h"
# include "geometry_msgs/Twist.h"

int main(int argc, char** argv){

  ros::init(argc,argv,"publisher_test");
  ros::NodeHandle n;
  ros::Publisher publisher_test = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    geometry_msgs::Twist cmd;

    cmd.linear.x = 1;
    publisher_test.publish(cmd);
    ROS_INFO("%f", cmd.linear.x);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
