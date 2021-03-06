#!/usr/bin/env python

## sim_master.py

## MAE 4750: HW 1 
## Group: am_zl_av_4750
## Members: Anthony McNicoll, Andy Li, Alex Volkov

import rospy
from am_zl_av_4750_hw1.msg import *
from am_zl_av_4750_hw1.srv import *

#######################################################################################
# BEGIN INITIAL PREP

#Grab /num_blocks and /configuration parameters from ROS server
#import pdb; pdb.set_trace()
NUMBLOCKS = rospy.get_param('/num_blocks')
INITIAL_CONFIG = rospy.get_param('/configuration')
VALID_CONFIGS = ['scattered', 'stacked_ascending', 'stacked_descending']
VALID_CMDS = ['open', 'close', 'moveTo', 'moveOver']

rospy.loginfo('Initial configuration specified as %s' % INITIAL_CONFIG)

#Nth index is N+1th block, and value specifies number of block below
bBlock = [0] * NUMBLOCKS
hasObject = 0   # Initially no object in gripper
openGripper = True # Initially gripper is open

# Set up bBlock to match initial configuration
if INITIAL_CONFIG == 'scattered':
    pass
    # bBlock all zeros
elif INITIAL_CONFIG == 'stacked_ascending':
    bBlock = range(NUMBLOCKS)
elif INITIAL_CONFIG == 'stacked_descending': 
    bBlock = range(NUMBLOCKS)
    bBlock = [x + 2 for x in belowBlock]
    bBlock[-1] = 0
else:
    rospy.logwarn('Incorrect initial state specified, defaulting to scattered configuration')
    # bBlock all zeros

# END INITIAL PREP
#######################################################################################
# BEGIN FUNCTION DEFINITIONS

# Attempt to perform a state update given the action and target specified in the
# /move_robot service request. Return boolean to signify success or failure.
# AKA THIS IS WHERE THE MAGIC HAPPENS
# Entire state consists of:
# A vector of NUMBLOCKS size describing block connectivity
# Object presence flag
# Open gripper flag
def updateState(action, target):
    global openGripper
    global hasObject
    global bBlock

    rospy.loginfo("Attempting to carry out command %s on target %s" % (action, target))
    
    if str(action) == 'open':
        openGripper = True
        rospy.loginfo("Grip true")
        
    elif str(action) == 'close':
        openGripper = False
        rospy.loginfo("Grip false")

    # Note that a collision (bringing cube to another cube) is not allowed for
    elif str(action) == 'moveTo':
        if openGripper:
            hasObject = target
        rospy.loginfo("MoveTo")

    elif str(action) == 'moveOver':
        if (hasObject > 0) and (openGripper is False):
            bBlock[hasObject - 1] = target
        rospy.loginfo("MoveOver")

    return True

    
# /move_robot service request handler.
def handle_move_robot(request):
    
    # Check if action is an open or close command, in which case target must be 0
    if (request.action in VALID_CMDS[:2]) and (request.target == 0):
        # Valid arguments -- attempt to update state, results in boolean
        success = updateState(request.action, request.target)
    # Check if action is a move command, in which case target must be a valid block number (1 to NUMBLOCKS)
    elif (request.action in VALID_CMDS[2:4]) and (request.target in range(NUMBLOCKS+1)):
        # Valid arguments -- attempt to update state, results in boolean
        success = updateState(request.action, request.target)
    # If argument or target are invalid
    else:
        # Yell at us and return False on service /move_robot
        rospy.logwarn("Invalid command on /move_robot:" +  str(request.action) + str(request.target))
        success = False

    # return result as node's response to /move_robot service request
    return MoveRobotResponse(success)

def sim_master():  

    # Start up the node, give it a name (literally tells ROS Master it exists)
    rospy.init_node('sim_master')
    
    # Create connection to /state topic and create publish object 
    pubState = rospy.Publisher('state', State, queue_size=10)

    # Connect to /move_robot service as server, call handle_move_robot() as handler to request
    rospy.Service('move_robot', MoveRobot, handle_move_robot)
    
    # Set the loop rate to 1Hz (used below)
    rate = rospy.Rate(1)
    
    # Some debug to let us know we're okay
    rospy.loginfo("Prep work complete, sim_master is online! [%s]" % rospy.get_time())

    # Continuously publish to /state at 1Hz
    while not rospy.is_shutdown():

        # Publish the state variables
        pubState.publish(belowBlock=bBlock,hasObj=hasObject,openGrip=openGripper) 

        # And sleep as long as needed to meet 1Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        sim_master()
    except rospy.ROSInterruptException:
        pass
