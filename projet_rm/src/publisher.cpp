# include "ros/ros.h"
# include "std_msg/String.h"

ros::init(argc,argv,"publisher");
ros::NodeHandle n;
ros::Publisher publisher_test = n.advertise<std_msg::String>("publisher",1000);
ros::Rate loop_rate(10);

int count = 0;
while(ros::ok()){
  std_msg::String msg;

  std::stringstream ss;
  ss<<"hello world "<<count;
  msg.data = ss.str();
  publisher_test.publish(msg);
  ROS_INFO("%s", msg.data.c_str());
  ros::spinOnce();
  loop_rate.sleep();
}
