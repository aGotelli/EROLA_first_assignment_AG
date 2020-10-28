## @package state_machine
#   \file state_machine.py
#   \brief This file contains the state machine for the high level control of this application.
#   \author Andrea Gotelli
#   \version 0.2
#   \date 22/10/2020
#
#   \param [in] world_width the width of the discretized world
#   \param [in] world_height the height of the discretized world
#   \param [in] sleep_x_coord is the x coordinate of the position where the robot sleeps
#   \param [in] sleep_y_coord is the y coordinate of the position where the robot sleeps
#
#   \details
#
#   Subscribes to: <BR>
#       ° /PlayWithRobot topic where the person publishes the given play command
#
#   Publishes to: <BR>
#       ° [None]
#
#   Service : <BR>
#       ° /Gesture as client, it waits for the gesture to come.
#
#       ° /MoveToPosition as cliet, ask to simulate the motion to the given position
#
#   Description :
#
#   This file uses smach libraries to generate a state machine, which is used to control the
#   behaviors of the robot. Specifically, the state machine defines the transition from a state to
#   another using the interface provided by smach.
#
#   This file could be considered the head of the simulation an its more complex and important part.
#
#   The states are defined in the respective classes, and the transistions are automatically performed
#   by the state machine from the smach libraries.
#

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

##
#   \brief Define the width of the discretized world. It is a parameter defined by the user.
width = 0

##
#   \brief Define the height of the discretized world. It is a parameter defined by the user.
height = 0

##
#   \brief Is the position where the robot goes to sleep, it is a paremter defined by the user
sleep_station = Pose()

##
#   \brief Defines the maximum level of fatigue the robot can andle before going to sleep.
fatigue_threshold = 0

##
#   Service client defined as global to be acessible outside the main.
move_to_pos_client = rospy.ServiceProxy('/MoveToPosition', MoveTo)


##
#    \brief reachPosition calls the service /MoveToPosition and prints out a string.
#    \param pose [geometry_msgs/Pose] is the position the robot should reach.
#    \param info [string] is information to display when approaching the position.
#
#    This function calls the service /MoveToPosition and prints out a string. Its usage should
#    preferred insted of pasting the same lines of code around.
def reachPosition(pose, info):
    rospy.wait_for_service('/MoveToPosition')
    try:
        print(info)
        move_to_pos_client(pose)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



##
#    \brief isTired check the level of fatigue to establish if the robot is "tired"
#    \param level [integer] is the current level of fatigue of the robot.
#    \return a boolen. True if the robot is "tired" false elsewhere.
#
#    This function compares the level of fatigue with and hard threshold which is a parameter
#    eccessible for the user.
#
def isTired(fatigue_level):
    if fatigue_level >= fatigue_threshold:
        return True
    else :
        return False

##
#   \class Move
#   \brief This class defines the state of the state machine corresponding to the robot randomly moving
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Move state. In this state the robot moves around randomly calling
#   the service /MoveToPosition. As part of the smach class, this class has the member function execute()
#   providing the intended behavior. For more details about the content of this class, see the member functions
#   documentation.
#
class Move(smach.State):
    ##
    #   \brief __init__ initializes the class and its members
    #   \param outcomes list of the possible outcomes from this state of the state machine.
    #   \param input_keys list of the possible input for this state (user data shared among states).
    #   \param output_keys list of the possible outputs for this state (user data shared among states).
    #
    #   This member function initializes the state machine state. It follows the conventions for smach.
    #   Moreover, it initializes the subscriber necessary to receive a command from the person.
    #
    def __init__(self):

            smach.State.__init__(self,
                                 outcomes=['tired','playing'],
                                 input_keys=['move_fatigue_counter_in', 'move_person_position_in'],
                                 output_keys=['move_fatigue_counter_out', 'move_person_position_out'])
            #   Definition of the ROS subscriber to account if the person wish to interact with the robot
            self.person_command = rospy.Subscriber("/PlayWithRobot", PersonCalling, self.commandReceived)
            #   Definition of the string containing what the person has commanded
            self.person_willing = "none"
            #   Definition of the person position using geometry_msgs/Pose
            self.person = Pose()
    ##
    #   \brief commandReceived is the callback for the ROS subscriber to the topic /PlayWithRobot
    #   \param command is the command received from the person willing to interact with the robot.
    #
    #   This member function checks the type of command that is received (only play is awailable in
    #   this context but other commands could be configurated). After it stores the position of the person
    #   willing to interact with the robot and it prints a log informing about what was received.
    #
    def commandReceived(self, received_person):
         #   Print a information about the received message
        print('Received command : ', received_person.command.data, ' from a person in : ', received_person.position.position.x, ', ' , received_person.position.position.y)
        #   Store the information about the received command
        self.person_willing = received_person.command.data
        #   Store the person position
        self.person.position = received_person.position.position

    ##
    #   \brief execute is main member function of the class, containing the intended behavior
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This member function is responsible of simulating the Move behavior for the robot.
    #   First it checks if the person has commanded something, in which case it returns the state 'playing'
    #   in order to change the state into Play.
    #   Secondly it checks if the robot is "tired". In a positive case it returns a string containing 'tired'
    #   to make the state machine to move to the Rest state.
    #   Finally, if none of the previous conditions has occurred, it increses the fatigue counter, it generates
    #   a random postion and it calls the service /MoveToPosition.
    #
    #   This member function loops in the previusly described phases untill one of the first two cases appears.
    #
    def execute(self, userdata):
        #   Main loop
        while not rospy.is_shutdown():

            #   Check if the person has commanded play
            if self.person_willing == "play" :
                #   Prepare the userdata to share the person position
                userdata.move_person_position_out = self.person
                #   Reset the person willing
                self.person_willing  = "none"
                #   Return 'plying' to change the state
                return 'playing'
            else :
                #   Check is the robot is tired
                if isTired(userdata.move_fatigue_counter_in) :
                    #   Print a log to inform about the fact that the robot is tired
                    print('Robot is tired of moving...')
                    #   Return 'tired' to change the state
                    return 'tired'
                else :
                    #   If none of the previous was true, then continue with the Move behavior
                    #   Declare a geometry_msgs/Pose for the random position
                    random_ = Pose()
                    #   Define the random components (x, y) of this random position
                    random_.position.x = random.randint(0, width)
                    random_.position.y = random.randint(0, height)
                    #   Call the service to reach this position
                    reachPosition(random_, 'Moving to a random position')
                    #   Increment the level of the robot fatigue
                    userdata.move_fatigue_counter_out = userdata.move_fatigue_counter_in + 1
                    #   Print a log to show the level of fatigue
                    print('Level of fatigue : ', userdata.move_fatigue_counter_in)



##
#   \class Rest
#   \brief This class defines the state of the state machine corresponding to the robot sleeping.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Rest state. Where the pet like robot rests for recover energies.
#   As part of the smach class, this class has the member function execute() providing the intended
#   behavior. For more details about the content of this class, see the member function documentation.
#
class Rest(smach.State):
    ##
    #   \brief The __init__ constructor initializes the state outcome and input output keys
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['rested'],
                                 input_keys=['rest_fatigue_counter_in'],
                                 output_keys=['rest_fatigue_counter_out'])
    ##
    #   \brief  The member function executing the state behavior
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This simple member function moves the robot in the sleeping position and waits for some time
    #   in order to simulate the pet like robot sleeping. It also resets the fatigue level to zero
    #   to let the robot to move and performs his behaviors.
    #
    def execute(self, userdata):
        #   Call the service to reach the position corresponding to the sleeping position
        reachPosition(sleep_station, 'Going to sleep...')
        #   Sleep for some time
        print('Sleeping...')
        rospy.sleep(10)
        print('Woke up!')
        #   Reset the fatigue counter afther the robot is well rested
        userdata.rest_fatigue_counter_out = 0
        #   Return 'rested' to change the state
        return 'rested'

##
#   \class Play
#   \brief This class defines the state of the state machine corresponding to the robot playing.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Play state. In this state the robot first goes to the
#   person postion, then, it waits for the person gesture.
#   This is done by calling the services /MoveToPosition and /GiveGesture. When a position
#   is returned, it moves to that position and it goes back to the person, waiting for another
#   position to reach.
#   As part of the smach class, this class has the member function execute() providing the intended behavior.
#   For more details about the content of this class, see the member function documentation.
#
class Play(smach.State):

    ##
    #   \brief __init__ is the constructor for the class.
    #
    #   This constructor initializes the outcomes and the input outout keys for this staste.
    #   Additionally, it initializes a service client used to call the /Gesture server for
    #   obtaining a gesture from the person
    #
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['tired','stop_play'],
                                 input_keys=['play_fatigue_counter_in', 'play_person_position_in'],
                                 output_keys=['play_fatigue_counter_out', 'play_person_position_out'])
            self.wait_for_gesture = rospy.ServiceProxy('/Gesture', GiveGesture)
    ##
    #   \brief This function performs the behavior for the state
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This function performs the Play behavior wich consist of interaction with the person.
    #   It moves the robot in order to reach the person position and it waits for gesture,
    #   calling the service /Gesture. This procedure is done for n times, with n randomly
    #   varying between 1 and 4
    #   Each time the robot moves it increases the level of fatigue and it checks if it
    #   is equal to the maximum.
    #
    def execute(self, userdata):
        #   Call the service to reach the position where the person is
        reachPosition(userdata.play_person_position_in, 'Going to person position')
        #   Increment the counter for the fatigue as the robot has moved
        userdata.play_fatigue_counter_out = userdata.play_fatigue_counter_in + 1
        #   Print a log showing the current level of fatigue
        print('Level of fatigue : ', userdata.play_fatigue_counter_in)

        #   Random number of times to play between 1 and 4
        number_of_iterations = random.randint(1, 4)
        #   Iterate in the Play behavior
        for iteration in range(number_of_iterations):
            #   First check if the robot is tired
            if isTired(userdata.play_fatigue_counter_in) :
                #   Print a log to inform about this event
                print('Robot is tired of playing...')
                #   Return 'tired' to change the state
                return 'tired'

            #   Else if the robot is not tired, wait for the gesture
            rospy.wait_for_service('/Gesture')
            try:
                gesture = self.wait_for_gesture()
                print('obtained', gesture.goal.position.x, gesture.goal.position.y)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            #   Call the service to reach the pointed position
            reachPosition(gesture.goal, 'Going to pointed position')
            #   Increase the fatigue counter as the robot haa moved
            userdata.play_fatigue_counter_out = userdata.play_fatigue_counter_in + 1
            print('Level of fatigue : ', userdata.play_fatigue_counter_in)
        return 'stop_play'

##
#   \brief __main__ intializes the ros node and the smach state machine
#
#   This functions does nothing more than initializing the node and the state machine.
#   It also retrieves the parameters from the ros parameters server and the user data
#   exchanged among the state machine states.
#

if __name__ == "__main__":
#    main()
#def main():
    #   Initialization of the ros node
    rospy.init_node('robot_behavior_state_machine')

    random.seed()

    #   Retrieve the parameter about the world dimensions
    width = rospy.get_param('world_width', 20)
    height = rospy.get_param('world_height', 20)

    #   Retrieve parameters about the sleeping position
    sleep_station.position.x = rospy.get_param('sleep_x_coord', 0)
    sleep_station.position.y = rospy.get_param('sleep_y_coord', 0)

    fatigue_threshold = rospy.get_param('fatigue_threshold', 5)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['behavior_interface'])
    sm.userdata.fatigue_level = 0
    sm.userdata.person = Pose()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Move(),
                               transitions={'tired':'REST',
                                            'playing':'PLAY'},
                               remapping={'move_fatigue_counter_in':'fatigue_level',
                                          'move_fatigue_counter_out':'fatigue_level',
                                          'move_person_position_out':'person',
                                          'move_person_position_in':'person'})

        smach.StateMachine.add('REST', Rest(),
                               transitions={'rested':'MOVE'},
                               remapping={'rest_fatigue_counter_in':'fatigue_level',
                                          'rest_fatigue_counter_out':'fatigue_level'})

        smach.StateMachine.add('PLAY', Play(),
                               transitions={'tired':'REST',
                                            'stop_play':'MOVE'},
                               remapping={'play_fatigue_counter_in':'fatigue_level',
                                          'play_fatigue_counter_out':'fatigue_level',
                                          'play_person_position_in':'person',
                                          'play_person_position_out':'person'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('robot_behavior_state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
