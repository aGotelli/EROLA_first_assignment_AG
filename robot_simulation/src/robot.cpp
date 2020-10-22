/**
 * \file robot.cpp
 * \brief This files emulates the a robot moving in the environment
 * \author Andrea Gotelli
 * \version 0.1
 * \date 22/10/2020
 *
 *
 *
 * Subscribes to: <BR>
 *    ° [None]
 *
 * Publishes to: <BR>
 *    ° [None]
 *
 * Service : <BR>
 *    ° /MoveToPosition
 *
 * Description :
 *        This node simulates a robot moving in the environment. The robot movements are simulated with
 *      the use of a service called MoveToPosition. This choice was to fullfill the request of having
 *      the robot to ingnore any other command while moving.
 *
 */


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "robot_simulation_messages/MoveTo.h"


/*!
 * \brief MoveToGivenPosition
 * \param T The target postion we want to reach
 * \return
 *
 * This function is the service callback. When it executed it waits for a specific amount of time
 * in order to simulate a robot moving to a derminate position.
 *
 * \todo Change the fixed time to a time that variates randomly
 *
 */
bool MoveToGivenPosition(robot_simulation_messages::MoveToRequest& T,
                          robot_simulation_messages::MoveToResponse&)
{
  ros::Duration waiting_time(3);
  waiting_time.sleep();
  ROS_INFO_STREAM("Position reached : " << T.goal.position.x << ", " << T.goal.position.y);
  return true;
}


/*!
 * \brief main Initilize the ros node and the service proder
 * \param argc
 * \param argv
 * \return
 */
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

