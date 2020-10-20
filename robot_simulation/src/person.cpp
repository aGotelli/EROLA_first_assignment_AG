#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>

#include <time.h>
#include <random>

#include "robot_simulation_messages/PersonCalling.h"




//geometry_msgs::Pose randPose(const int world_width, const int world_height)
//{
//  geometry_msgs::Pose position;
//  position.position.x = (rand()/RAND_MAX)*(world_width + 1);
//  position.position.y = (rand()/RAND_MAX)*(world_height + 1);

//  return position;
//}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "person");
  srand(time(nullptr));

  ros::NodeHandle nh_glob;

  int width, height;

  nh_glob.param("world_width", width, 20);
  nh_glob.param("world_height", height, 20);

  ros::Publisher command_pub = nh_glob.advertise<robot_simulation_messages::PersonCalling>("/PlayWithRobot", 10);

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
