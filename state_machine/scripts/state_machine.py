"""!
\file state_machine.py
\brief this file contains the state machine for the high level control of this application
\author Andrea Gotelli
\version 0.2
\date 22/10/2020

\param [in] world_width the width of the discretized world
\param [in] world_height the height of the discretized world
\param [in] sleep_x_coord is the x coordinate of the position where the robot sleeps
\param [in] sleep_y_coord is the y coordinate of the position where the robot sleeps

\details

Subscribes to: <BR>
    ° /PlayWithRobot topic where the person publishes the given play command

Publishes to: <BR>
    ° [None]

Service : <BR>
    ° /GiveGesture as client, it waits for the gesture to come.
    ° /MoveToPosition as cliet, ask to simulate the motion to the given position

Description :
        This file uses smach libraries to generate a state machine, which is used to control the
    behaviors of the robot. Specifically, the state machine defines the transition from a state to
    another using the interface provided by smach.

    This file could be considered the head of the simulation an its more complex and important part.

    The actions and the transitions are explained below:

        ° The robot starts with the Move behavior, which makes the robot to move randomly in
          the environment. In fact, a random postion (within the limit of the world) is
          generated and passed to the service /MoveToPosition which simulates the motion.

        ° When the robot is tired it goes in the Rest behavior. By doing this, it will first
          move to the sleeping postion and it will wait some time before waiking up. Here, again,
          to reach the position the service /MoveToPosition is called.

        ° When the person gives a command (throught the topic /PlayWithRobot) the robot reaches the
          position of the person (which is contained into the message) and waits for a gesture. The
          gesture is indeed provided by the service /GiveGesture.

"""


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

move_to_pos_client = rospy.ServiceProxy('/MoveToPosition', MoveTo)

def reachPosition(pose, info):
    """!
    \brief reachPosition calls the service /MoveToPosition and prints out a string.
    \param pose [geometry_msgs/Pose] is the position the robot should reach.
    \param info [string] is information to display when approaching the position.

    This function calls the service and prints out a string. It is used instead of
    pasting the same code around.
    """
    rospy.wait_for_service('/MoveToPosition')
    try:
        print(info)
        move_to_pos_client(pose)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


fatigue_threshold = 0
def isTired(fatigue_level):
    """!
    \brief isTired check the level of fatigue to establish if the robot is "tired"
    \param level [integer] is the current level of fatigue of the robot.
    \return a boolen. True if the robot is "tired" false elsewhere.

    This function compares the level of fatigue with and hard threshold defined within
    the function.
    """
    threshold = 5
    if fatigue_level >= fatigue_threshold:
        return True
    else :
        return False


sleep_station = Pose()

class Move(smach.State):
    """!
    \class Move
    \brief This class defines the state of the state machine corresponding to the robot randomly moving

    This class inheritates from smach and it consist of a state in the state machine. The state
    that is represented here is the Move state. In this state the robot moves around randomly calling
    the service /MoveToPosition. As part of the smach class this class has the member function \function execute
    providing the intended behavior. For more details about the content of this class, see the member function
    documentations.
    """

    def __init__(self):
            """!
            \brief __init__ initializes the class and its members.
            \param outcomes list of the possible outcomes from this state of the state machine.
            \param input_keys list of the possible input for this state (user data shared among states).
            \param output_keys list of the possible outputs for this state (user data shared among states).


                This member function initializes the state machine state. It follows the conventions for smach.
             Moreover, it initializes two members of the class corresponding of a subscriber and a service
             client. These two elements are necessary to simulate the robot moving and receive a command from
             the person.

             \todo connect functions names here
            """
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

    def commandReceived(self, received_person):
        """!
        \brief commandReceived is the callback for the ROS subscriber to the topic /PlayWithRobot
        \param command is the command received from the person willing to interact with the robot.


            This member function checks the type of command that is received (only play is awailable in
        this context but other commands could be configurated). After it stores the position of the person
        willing to interact with the robot and it prints a log informing about what was received.
        """
        #   Print a information about the received message
        print('Received command : ', received_person.command.data, ' from a person in : ', received_person.position.position.x, ', ' , received_person.position.position.y)
        #   Store the information about the received command
        self.person_willing = received_person.command.data
        #   Store the person position
        self.person.position = received_person.position.position


    def execute(self, userdata):
        """!
        \brief execute is main member function of the class, containing the intended behavior
        \param userdata is data exchanged among states.
        \return a string consisting of the state outcome

            This member function is responsible of simulating the Move behavior for the robot.
        First it checks if the person has commanded something, in which case it returns the state 'playing'
        in order to change the state into Play.
        Secondly it checks if the robot is "tired". In a positive case it returns a string containing 'tired'
        to make the state machine to move to the Rest state.
        Finally, if none of the previous conditions has occurred, it increses the fatigue counter, it generates
        a random postion and it calls the service /MoveToPosition.

        This member function loops in the previusly described phases untill one of the first two cases appears.
        """
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




class Rest(smach.State):
    """!
    \class Rest
    \brief This class defines the state of the state machine corresponding to the robot sleeping.

    This class inheritates from smach and it consist of a state in the state machine. The state
    that is represented here is the Rest state. In this state the robot first goes to the sleeping
    postion (calling the service /MoveToPosition). Then it resets the fatigue level to zero, simulating
    the robot recover of energies after having sleep.
    """
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['rested'],
                                 input_keys=['rest_fatigue_counter_in'],
                                 output_keys=['rest_fatigue_counter_out'])

    def execute(self, userdata):
        #   Call the service to reach the position corresponding to the sleeping position
        reachPosition(sleep_station, 'Going to sleep...')
        #   Reset the fatigue counter afther the robot is well rested
        userdata.rest_fatigue_counter_out = 0
        #   Return 'rested' to change the state
        return 'rested'


class Play(smach.State):
    """!
    \class Play
    \brief This class defines the state of the state machine corresponding to the robot playing.

    This class inheritates from smach and it consist of a state in the state machine. The state
    that is represented here is the Play state. In this state the robot first goes to the
    person postion, then, it waits for the person gesture.
    This is done by calling the services /MoveToPosition and /GiveGesture. When a position
    is returned, it moves to that position and it goes back to the person, waiting for another
    position to reach.

    This procedure is done for three times

    \todo change the nuber of times into a random variable.
    """
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['tired','stop_play'],
                                 input_keys=['play_fatigue_counter_in', 'play_person_position_in'],
                                 output_keys=['play_fatigue_counter_out', 'play_person_position_out'])
            self.wait_for_gesture = rospy.ServiceProxy('/GiveGesture', GiveGesture)

    def execute(self, userdata):
        #   Call the service to reach the position where the person is
        reachPosition(userdata.play_person_position_in, 'Going to person position')
        #   Increment the counter for the fatigue as the robot has moved
        userdata.play_fatigue_counter_out = userdata.play_fatigue_counter_in + 1
        #   Print a log showing the current level of fatigue
        print('Level of fatigue : ', userdata.play_fatigue_counter_in)

        #number_of_iterations = 3
        #   Random number of times to play between 1 and 4
        number_of_iterations = randint(1, 4)
        #   Iterate in the Play behavior
        for iteration in range(number_of_iterations):
            #   First check if the robot is tired
            if isTired(userdata.play_fatigue_counter_in) :
                #   Print a log to inform about this event
                print('Robot is tired of playing...')
                #   Return 'tired' to change the state
                return 'tired'

            #   Else if the robot is not tired, wait for the gesture
            rospy.wait_for_service('/GiveGesture')
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


if __name__ == "__main__":
    """!
    \brief __main__ intializes the ros node and the smach state machine

        This functions does nothing more than initializing the node and the state machine.
    It also retrieves the parameters from the ros parameters server and the user data
    exchanged among the state machine states.


    """
    #   Initialization of the ros node
    rospy.init_node('robot_behavior_state_machine')

    #   Retrieve the parameter about the world dimensions
    width = rospy.get_param('world_width', 20)
    height = rospy.get_param('world_height', 20)

    #   Retrieve parameters about the sleeping position
    sleep_station.position.x = rospy.get_param('sleep_x_coord', 0)
    sleep_station.position.y = rospy.get_param('sleep_y_coord', 0)

    fatigue_threshold = rospy.get_param('fatigue_threshold', 5)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['behavior_interface'])
    sm.userdata.sm_counter = 0
    sm.userdata.person = Pose()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Move(),
                               transitions={'tired':'REST',
                                            'playing':'PLAY'},
                               remapping={'move_fatigue_counter_in':'sm_counter',
                                          'move_fatigue_counter_out':'sm_counter',
                                          'move_person_position_out':'person',
                                          'move_person_position_in':'person'})

        smach.StateMachine.add('REST', Rest(),
                               transitions={'rested':'MOVE'},
                               remapping={'rest_fatigue_counter_in':'sm_counter',
                                          'rest_fatigue_counter_out':'sm_counter'})

        smach.StateMachine.add('PLAY', Play(),
                               transitions={'tired':'REST',
                                            'stop_play':'MOVE'},
                               remapping={'play_fatigue_counter_in':'sm_counter',
                                          'play_fatigue_counter_out':'sm_counter',
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

