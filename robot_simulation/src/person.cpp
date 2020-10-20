#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "person");
  ros::NodeHandle nh;

  ros::Publisher command_pub = nh.advertise<std_msgs::String>("/PlayWithRobot", 10);

  ros::Rate loop_rate(50);

  ros::Time current_time = ros::Time::now();
  ros::Time prev_time = current_time;
  ros::Duration time_elapsed;
  while (ros::ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();
    time_elapsed = current_time - prev_time;
    if( time_elapsed.toSec() >= 10 ) {

      std_msgs::String msg;
      msg.data = "play";

      command_pub.publish(msg);
      prev_time = ros::Time::now();
      ROS_INFO_STREAM("commanding play");
    }


    loop_rate.sleep();
  }

  return 0;
}
