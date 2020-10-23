/**
 * \file person.cpp
 * \brief This files emulates the a person giving commands to a robot
 * \author Andrea Gotelli
 * \version 0.1
 * \date 20/10/2020
 *
 * \param [in] world_width
 * \param [in] world_height
 *
 * \details
 *
 * Subscribes to: the topic where the state machine publishes the current state
 *    ° /robot_behavior_state_machine/smach/container_status
 *
 * Publishes to: <BR>
 *    ° /PlayWithRobot
 *
 * Service : <BR>
 *    ° /GiveGesture
 *
 * Description :
 *        This node simulates a person behavior. It publishes the command the person wants to give in
 *      the topic: /PlayWithRobot. Before publishing the command, it check that the robot is neither
 *      in the Rest nor in the Play behavior already. This is done by comparing the state that has been
 *      published by the state machine.
 *      Moreover, it simulates a person taking time to point a position
 *      to the robot. This is done by a service, with waits for some time to give a random position.
 *
 */



#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>

#include <time.h>
#include <random>

#include "robot_simulation_messages/PersonCalling.h"
#include "robot_simulation_messages/GiveGesture.h"
#include <smach_msgs/SmachContainerStatus.h>


static int width;   ///< World discretized dimension in width.
static int height;  ///< World discretized dimension in height.


/*!
 * \brief PointingGesture it is the service callback.
 * \param gesture it is the pointed position.
 * \return always true as this method cannot fail.
 *
 * This function creates a geometry_msgs/Pose message. It fills up the message
 * with random value for x and y (ranging between 0 and the width, or height, respectively)
 */
bool PointingGesture(robot_simulation_messages::GiveGesture::Request&,
                      robot_simulation_messages::GiveGesture::Response& gesture)
{
  //  Create the pointed location as a random position
  gesture.goal.position.x = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(width + 1));
  gesture.goal.position.y = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(height + 1));

  //  Log of the intention of chosing the position
  ROS_INFO_STREAM("Deciding location to point...");

  //  Wait to simulate the motion
  ros::Duration waiting_time(3);
  waiting_time.sleep();

  //  Log to inform that position has been chosen
  ROS_INFO_STREAM("Pointed location decided");
  return true;
}

//  Static container to store the callback argument
static std::string state_machine_status;

/*!
 * \brief SaveStatus is the callback for the subscriber of the topic /robot_behavior_state_machine/smach/container_status
 * \param state_machine_status_ is the message containing the stateus of the state machine
 *
 * This functions does nothing but saving the state that was received in the message. The state is expressed
 * by a string that can be: MOVE, PLAY or REST;
 */
void SaveStatus(smach_msgs::SmachContainerStatus::ConstPtr state_machine_status_)
{
  state_machine_status = state_machine_status_->active_states.front();
}


/*!
 * \brief main menage to collect parameters and pubishes a command constantly
 * \param argc
 * \param argv
 * \return
 *
 *  The main function first collect all the parameters from the ros server. Then it initialises
 * the publisher. The publisher emulates a person who call the robot to play. The person is assumed
 * to move randomly in the environment.
 *
 * \todo Change te regularity in calling with a random amount of time.
 */
int main(int argc, char **argv)
{

  //  Initialize ros node
  ros::init(argc, argv, "person");

  //  Initialize for random number generation
  srand(time(nullptr));

  //  Declare a global node handler
  ros::NodeHandle nh_glob;

  //  Retireve the world prameters
  nh_glob.param("world_width", width, 20);
  nh_glob.param("world_height", height, 20);

  //  Definition the ROS publisher to publish the command to the robot
  ros::Publisher command_pub = nh_glob.advertise<robot_simulation_messages::PersonCalling>("/PlayWithRobot", 10);

  //  Definition of the subscriber to obtain the current state of the state machine
  ros::Subscriber state_machine_status_sub = nh_glob.subscribe<smach_msgs::SmachContainerStatus>("/robot_behavior_state_machine/smach/container_status", 1, SaveStatus);

  //  Definition of the service provider to provide the robot of a random pointed position.
  ros::ServiceServer give_gesture = nh_glob.advertiseService("/GiveGesture", PointingGesture);

  //  Definition of the frame rate for this node
  ros::Rate loop_rate(50);

  //  Time handle section
  ros::Time current_time = ros::Time::now();
  ros::Time prev_time = current_time;
  ros::Duration time_elapsed;

  //  Main loop
  while (ros::ok())
  {
    //  Obtain published messages
    ros::spinOnce();

    //  Compute elapsed time
    current_time = ros::Time::now();
    time_elapsed = current_time - prev_time;
    prev_time = ros::Time::now();

    //  Check that deseired time has elapsed and the robot is in Move state (neither in Rest not in Play)
    if( time_elapsed.toSec() >= 10 && state_machine_status == "MOVE") {

      //  Declaration of the command to publish to the robot
      robot_simulation_messages::PersonCalling person;

      //  Definition of the given command
      person.command.data = "play";

      //  Definition of the random position where the person calls the robot to play
      person.position.position.x = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(width + 1));
      person.position.position.y = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(height + 1));

      //  Publish the information to the robot
      ROS_INFO_STREAM("A person is commanding play");
      command_pub.publish(person);


    }

    loop_rate.sleep();
  }

  return 0;
}
