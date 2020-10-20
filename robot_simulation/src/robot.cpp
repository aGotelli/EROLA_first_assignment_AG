#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "robot_simulation_messages/MoveTo.h"

bool MoveToGivenPosition(robot_simulation_messages::MoveToRequest&,
                          robot_simulation_messages::MoveToResponse&)
{
  ROS_INFO_STREAM("calling the server");
  ros::Duration waiting_time(3);
  waiting_time.sleep();
  return true;
}

//void move1(const geometry_msgs::Pose::ConstPtr target)
//{
//  ros::Duration waiting_time(3);
//  waiting_time.sleep();
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot");
  ros::NodeHandle nh;

  ros::ServiceServer reach_position = nh.advertiseService("/MoveToPosition", MoveToGivenPosition);
//  ros::Subscriber receivedTarget = nh.subscribe<geometry_msgs::Pose>("/MoveToPosition", 10, move1);

  ros::Rate frame_rate = ros::Rate(50.0);

  while(ros::ok()) {
    ros::spinOnce();

    frame_rate.sleep();
  }
}

