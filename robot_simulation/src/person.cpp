/**
 * \file person.cpp
 * \brief This files emulates the a person giving commands to a robot
 * \author Andrea Gotelli
 * \version 0.1
 * \date 20/10/2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    ° [None]
 *
 * Publishes to: <BR>
 *    ° /PlayWithRobot
 *
 * Service : <BR>
 *    ° /GiveGesture
 *
 * Description :
 *        This node simulates a person behavior. It publishes the command the person wants to give in
 *      the topic: /PlayWithRobot. Moreover, it simulates a person taking time to point a position
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
  gesture.goal.position.x = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(width + 1));
  gesture.goal.position.y = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(height + 1));
  ROS_INFO_STREAM("Moving to pointed location");
  ros::Duration waiting_time(3);
  waiting_time.sleep();

  return true;
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
  ros::init(argc, argv, "person");
  srand(time(nullptr));

  ros::NodeHandle nh_glob;

  nh_glob.param("world_width", width, 20);
  nh_glob.param("world_height", height, 20);

  ros::Publisher command_pub = nh_glob.advertise<robot_simulation_messages::PersonCalling>("/PlayWithRobot", 10);

  ros::ServiceServer give_gesture = nh_glob.advertiseService("/GiveGesture", PointingGesture);
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

      robot_simulation_messages::PersonCalling command;
      command.command.data = "play";

      command.position.position.x = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(width + 1));
      command.position.position.y = static_cast<int>(( static_cast<double>(rand())/RAND_MAX)*(height + 1));

      command_pub.publish(command);
      prev_time = ros::Time::now();
      ROS_INFO_STREAM("commanding play");
    }


    loop_rate.sleep();
  }

  return 0;
}
