# include "ros/ros.h"
# include "geometry_msgs/Twist.h"
# include "geometry_msgs/Pose.h"
# include "geometry_msgs/Point.h"
# include <cmath>
# include "nav_msgs/Odometry.h"
# include "planification/ListePoints.h"


static float K = 0.2;
static float dist_p = 0.5;


geometry_msgs::Pose pos ;
geometry_msgs::Point checkpoint ;

void positionCallback(const nav_msgs::Odometry msg){
  pos = msg.pose.pose;
}

void objectiveCallback(const geometry_msgs::Point msg){
  checkpoint = msg;
}

int main(int argc, char** argv){

  ros::init(argc,argv,"commande");
  ros::NodeHandle n;
  ros::Publisher commande_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  ros::Subscriber position_sub = n.subscribe("odom",1000,positionCallback);
  ros::Subscriber checkpoint_sub = n.subscribe("checkpoints",1000,positionCallback);
  ros::Rate loop_rate(10);
  checkpoint.x = 5;
  checkpoint.y = 5;

  while(ros::ok()){

    //conversion quaternions -> lacet
    double siny_cosp = 2*(pos.orientation.w*pos.orientation.z + pos.orientation.x*pos.orientation.y);
    double cosy_cosp = 1-2*(pos.orientation.y*pos.orientation.y + pos.orientation.z*pos.orientation.z);
    float theta = atan2(siny_cosp,cosy_cosp);
    ROS_INFO("x : %f", pos.position.x);
    ROS_INFO("y : %f", pos.position.y);
    ROS_INFO("theta : %f", theta);

    float err_x = checkpoint.x - pos.position.x;
    float err_y = checkpoint.y - pos.position.y;

    float v_des_x = K*err_x;
    float v_des_y = K*err_y;

    float u_des_x = cos(theta)*v_des_x + sin(theta)*v_des_y;
    float u_des_w = -sin(theta)/dist_p*v_des_x + cos(theta)/dist_p*v_des_y;

    geometry_msgs::Twist cmd;
    cmd.linear.x = u_des_x;
    cmd.angular.z = u_des_w;
    commande_pub.publish(cmd);
    ROS_INFO("commande vx : %f", cmd.linear.x);
    ROS_INFO("commande w : %f", cmd.angular.z);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
