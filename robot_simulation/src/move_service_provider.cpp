/**
 * \file move_service_provider.cpp
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
 * \brief MoveToGivenPosition is the callback for the service emulating the robot movement.
 * \param T The target postion we want to reach
 * \return
 *
 * This function is the service callback. When it executed it waits for some time
 * in order to simulate a robot moving to a derminate position. The time varies randomly
 * in an interval from 3 to 6 seconds.
 *
 *
 */
bool MoveToGivenPosition(robot_simulation_messages::MoveToRequest& T,
                          robot_simulation_messages::MoveToResponse&)
{
  //  Wait for some time to simulate the robot moving to the position
  int min_time = 3;
  int max_time = 6;
  int travelling_time = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(max_time - min_time + 1) + min_time);
  ros::Duration waiting_time(travelling_time);
  waiting_time.sleep();

  //  Log the information that the position is reached
  ROS_INFO_STREAM("Position reached : " << T.goal.position.x << ", " << T.goal.position.y << " it took : " << travelling_time << " [s].");
  return true;
}


/*!
 * \brief main Initilize the ros node and the service proder
 * \param argc
 * \param argv
 *
 */
int main(int argc, char **argv)
{
  //  Initialize the ROS node
  ros::init(argc, argv, "move_service_provider");

  //  Declaration of a global node handle
  ros::NodeHandle nh_glob;

  //  Definition of the service provider to simulate the robot moving
  ros::ServiceServer reach_position = nh_glob.advertiseService("/MoveToPosition", MoveToGivenPosition);

  //  Definition of the node frame rate
  ros::Rate frame_rate = ros::Rate(50.0);

  //  Main loop
  while(ros::ok()) {

    //  Check for new messages
    ros::spinOnce();

    //  Waits
    frame_rate.sleep();
  }

  return 0;
}

