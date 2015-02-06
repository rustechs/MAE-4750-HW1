#!/usr/bin/env python

## controller.py

## MAE 4750: HW 1 
## Group: am_zl_av_4750
## Members: Anthony McNicoll, Andy Li, Alex Volkov

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
def stateCallback(data):
    

def controller():  
    
    # Start up the node, get it a name (literally tells ROS Master it exists)
    rospy.init_node('controller')
    
    # Wait here until /move_robot service becomes available
    # Effectively waits for sim_master node to come online
    rospy.wait_for_service('move_robot')

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
