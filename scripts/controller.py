#!/usr/bin/env python

## controller.py

## MAE 4750: HW 1 
## Group: am_zl_av_4750
## Members: Anthony McNicoll, Andy Li, Alex Volkov

import sys
import rospy
from am_zl_av_4750_hw1.msg import *
from am_zl_av_4750_hw1.srv import *
from std_msgs.msg import String 

#######################################################################################
# BEGIN INITIAL PREP

#Grab /num_blocks and /configuration parameters from ROS server
NUMBLOCKS = rospy.get_param('/num_blocks')
INITIAL_CONFIG = rospy.get_param('/configuration')
VALID_CONFIGS = ['scattered', 'stacked_ascending', 'stacked_descending']

# Initialize targetConfig to INITIAL_CONFIG from ROS Param server
# Perform validity check in the process
if INITIAL_CONFIG in VALID_CONFIGS:
    targetConfig = INITIAL_CONFIG
else:
    rospy.logwarn('Incorrect initial state specified, defaulting to scattered configuration')
    targetConfig = 'scattered'

sAsc = range(NUMBLOCKS)
sDesc = [x + 2 for x in sAsc]
sDesc[-1] = 0
configDict = {'scattered': [0]*NUMBLOCKS, 
              'stacked_ascending': sAsc,
              'stacked_descending': sDesc}

rospy.loginfo('Initial configuration specified as %s' % INITIAL_CONFIG)

# A "scattered" flag, because we are lazy.
scattered = False

# END INITIAL PREP
#######################################################################################
# BEGIN FUNCTION DEFINITIONS

# Deal with incoming /command topic messages
def cmdCallback(cmd):

    global targetConfig
    global scattered
    if cmd.data in VALID_CONFIGS:
        rospy.loginfo("Target state set to: %s." % cmd.data)
        targetConfig = cmd.data
        scattered = False
    else:
        rospy.logwarn("Incorrect command: %s. No action taken" % cmd.data)

# Deal with incoming /state topic messages
# On every update, check if system state matches intended state
# If not, send next required command
# Otherwise, do nothing
# AKA THIS IS WHERE THE MAGIC HAPPENS
def stateCallback(data):
    global moveRobot
    global scattered
    bBlock = [ ord(i) for i in data.belowBlock ]
    bBlockT = configDict[targetConfig]
#    import pdb; pdb.set_trace()

    # First step is to check if mission accomplished.
    if configDict[targetConfig] == bBlock: 
        rospy.loginfo("Achievement get: " + str(configDict[targetConfig]))

    # If not, take action depending on whether scattering has been achieved
    else:
        try:

            # If scattering has not been achieved, remove any top (not listed, not 0)
            #import pdb; pdb.set_trace()
            if not scattered:
                scattered = True
                for i in range(NUMBLOCKS):
                    if not (((i+1) in bBlock) or (bBlock[i] == 0)):
                        rospy.loginfo("Putting " + str(i+1) + ' on the floor.')
                        moveRobot('moveTo', i+1)
                        moveRobot('close', 0)
                        moveRobot('moveOver', 0)
                        moveRobot('open', 0)
                        scattered = False

            # If scattering has been achieved, stack in order
            # We've made sure it's scattered, so do everything at once! Lazy!
            else:
                lastBlock = 0
                for i in range(NUMBLOCKS):
                    thisBlock = bBlockT.index(lastBlock) + 1
                    rospy.loginfo("Putting " + str(thisBlock) + ' on ' + str(lastBlock))
                    moveRobot('moveTo', thisBlock)
                    moveRobot('close', 0)
                    moveRobot('moveOver', lastBlock)
                    moveRobot('open', 0)
                    lastBlock = thisBlock

        except rospy.ServiceException, e:
            raise e
            rospy.logfatal("Womp-womp. Service call failed: %s" % e)
            sys.exit(1)

#    import pdb; pdb.set_trace()

def controller(): 
    global moveRobot
    
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
    except:
        import pdb; pdb.set_trace()
