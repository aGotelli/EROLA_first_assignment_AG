#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import roslib
import rospy
import smach
import smach_ros
import time
import random

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from robot_simulation_messages.srv import MoveTo
from robot_simulation_messages.msg import PersonCalling
from robot_simulation_messages.srv import GiveGesture

width = 0
height = 0

def tired(level):
    threshold = 3
    if level > threshold:
        return True
    else :
        return False


class Position:
    def __init__(self):
        self.x = 0
        self.y = 0

sleep_station = Pose()

class Move(smach.State):
     def __init__(self):
             smach.State.__init__(self,
                                  outcomes=['tired','playing'],
                                  input_keys=['move_counter_in'],
                                  output_keys=['move_counter_out'])
             #self.give_target = rospy.Publisher('/MoveToPosition', Pose, queue_size=10)
             self.reach_position = rospy.ServiceProxy('/MoveToPosition', MoveTo)
             self.received_command = rospy.Subscriber("/PlayWithRobot", PersonCalling, self.callback)
             self.play = False
             self.person_position = Position()

     def callback(self, command):
         if command.command.data == "play" :
             self.play = True
             self.person_position.x = command.position.position.x
             self.person_position.y = command.position.position.y
             print('Received command from a person in : ', self.person_position.x, ', ' , self.person_position.y)


     def execute(self, userdata):
        while not rospy.is_shutdown():
            if self.play :
                self.play  = False
                return 'playing'

            if tired(userdata.move_counter_in) :
                print('Robot is tired of moving...')
                return 'tired'
            else :
                userdata.move_counter_out = userdata.move_counter_in + 1
                t = Pose()
                t.position.x = random.randint(0, width)
                t.position.y = random.randint(0, height)

                rospy.wait_for_service('/MoveToPosition')
                try:
                    print('Moving to a random position')
                    self.reach_position(t)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)




class Rest(smach.State):
     def __init__(self):
             smach.State.__init__(self,
                                  outcomes=['rested'],
                                  input_keys=['rest_counter_in'],
                                  output_keys=['rest_counter_out'])
             self.reach_position = rospy.ServiceProxy('/MoveToPosition', MoveTo)

     def execute(self, userdata):
         rospy.wait_for_service('/MoveToPosition')
         try:
             print('Going to sleep...')
             self.reach_position(sleep_station)
         except rospy.ServiceException as e:
             print("Service call failed: %s"%e)
         userdata.rest_counter_out = 0
         return 'rested'

class Play(smach.State):
     def __init__(self):
             smach.State.__init__(self,
                                  outcomes=['tired','stop_play'],
                                  input_keys=['play_counter_in'],
                                  output_keys=['play_counter_out'])
             self.wait_for_gesture = rospy.ServiceProxy('/GiveGesture', GiveGesture)

     def execute(self, userdata):

        for iteration in range(3):
            if tired(userdata.play_counter_in) :
                print('Robot is tired of playing...')
                return 'tired'

            rospy.wait_for_service('/GiveGesture')
            try:
                gesture = self.wait_for_gesture()
                print('obtained', gesture.goal.position.x, gesture.goal.position.y)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        return 'stop_play'


if __name__ == "__main__":

    rospy.init_node('robot_behavior_state_machine')

    width = rospy.get_param('world_width', 20)
    height = rospy.get_param('world_height', 20)

    sleep_station.position.x = rospy.get_param('sleep_x_coord', 0)
    sleep_station.position.y = rospy.get_param('sleep_y_coord', 0)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['behavior_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Move(),
                               transitions={'tired':'REST',
                                            'playing':'PLAY'},
                               remapping={'move_counter_in':'sm_counter',
                                          'move_counter_out':'sm_counter'})

        smach.StateMachine.add('REST', Rest(),
                               transitions={'rested':'MOVE'},
                               remapping={'rest_counter_in':'sm_counter',
                                          'rest_counter_out':'sm_counter'})

        smach.StateMachine.add('PLAY', Play(),
                               transitions={'tired':'REST',
                                            'stop_play':'MOVE'},
                               remapping={'play_counter_in':'sm_counter',
                                          'play_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

