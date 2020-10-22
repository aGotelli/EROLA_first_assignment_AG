#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "robot_simulation_messages/MoveTo.h"

bool MoveToGivenPosition(robot_simulation_messages::MoveToRequest& T,
                          robot_simulation_messages::MoveToResponse&)
{
  ros::Duration waiting_time(3);
  waiting_time.sleep();
  ROS_INFO_STREAM("Position reached : " << T.goal.position.x << ", " << T.goal.position.y);
  return true;
}



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

