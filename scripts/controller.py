#!/usr/bin/env python

## controller.py

## MAE 4750: HW 1 
## Group: am_zl_av_4750
## Members: Anthony McNicoll, Andy Li, Alex Volkov

import sys
import rospy
from am_zl_av_4750.msg import *
from am_z1_av_4750.srv import *
from std_msgs.msg import String 

#######################################################################################
# BEGIN INITIAL PREP

#Grab /num_blocks and /configuration parameters from ROS server
NUMBLOCKS = get_param('/num_blocks')
INITIAL_CONFIG = get_param('/configuration')
VALID_CONFIGS = ['scattered', 'stacked_ascending', 'stacked_descending']

# Initialize targetConfig to INITIAL_CONFIG from ROS Param server
# Perform validity check in the process
if INITIAL_CONFIG in VALID_CONFIGS:
    targetConfig = INITIAL_CONFIG
else
    rospy.logwarn('Incorrect initial state specified, defaulting to scattered configuration')
    targetConfig = 'scattered'

rospy.loginfo('Initial configuration specified as %s' % INITIAL_CONFIG)

# END INITIAL PREP
#######################################################################################
# BEGIN FUNCTION DEFINITIONS

# Deal with incoming /command topic messages
def cmdCallback(cmd):

    if cmd.data in VALID_CONFIGS:
        rospy.loginfo("Target state set to: %s." % cmd.data)
        targetConfig = cmd.data
    else:
        rospy.logwarn("Incorrect command: %s. No action taken" % cmd.data)

# Deal with incoming /state topic messages
# On every update, check if system state matches intended state
# If not, send next required command
# Otherwise, do nothing
# AKA THIS IS WHERE THE MAGIC HAPPENS
def stateCallback(data):
    
    # Blah blah -- the magic part. compare target to current state
    # Figure out next step toward target

    # won't be checked exactly like this, but something like this...
    if targetConfig != curConfig: 
        try:
            success = moveRobot(nextAction, nextBlock)
            if not success:
                rospy.logfatal("I'm afraid I can't do that, Dave")
                sys.exit(1)
        except rospy.ServiceException, e:
            rospy.logfatal("Service call failed: %s" % e)
            sys.exit(1)

def controller():  
    
    # Start up the node, get it a name (literally tells ROS Master it exists)
    rospy.init_node('controller')
    
    # Wait here until /move_robot service becomes available
    # Effectively waits for sim_master node to come online
    rospy.wait_for_service('move_robot')

    # Alias service call as a function
    moveRobot = rospy.ServiceProxy('move_robot', MoveRobot)

    # Register as subscriber on topic /state, handle incoming data with callback
    rospy.Subscriber('state', State, stateCallback)

    # Register as subscriber on topic /command, handle incoming commands with callback
    rospy.Subscriber('command', String, cmdCallback)

    # Some debug info to let us know we're okay
    rospy.loginfo("Prep work complete, controller is online! [%s]" % rospy.get_time())

    # Do nothing until the node is killed off
    # Yields to other threads
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
