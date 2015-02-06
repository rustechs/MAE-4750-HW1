#!/usr/bin/env python

## sim_master.py

## MAE 4750: HW 1 
## Group: am_zl_av_4750
## Members: Anthony McNicoll, Andy Li, Alex Volkov

import rospy
from am_zl_av_4750.msg import *
from am_z1_av_4750.srv import *

#######################################################################################
# BEGIN INITIAL PREP

#Grab /num_blocks and /configuration parameters from ROS server
NUMBLOCKS = get_param('/num_blocks')
INITIAL_CONFIG = get_param('/configuration')

rospy.loginfo('Initial configuration specified as %s' % INITIAL_CONFIG)

#Nth index is N+1th block, and value specifies number of block below
belowBlock = [0] * NUMBLOCKS

if INITIAL_CONFIG == 'scattered':
    # belowBlock all zeros
elif INITIAL_CONFIG == 'stacked_ascending':
    belowBlock = range(NUMBLOCKS)

elif INITIAL_CONFIG == 'stacked_descending': 
    belowBlock = range(NUMBLOCKS)
    belowBlock = [x + 2 for x in belowBlock]
    belowBlock = belowBlock.pop()
    belowBlock = belowBlock.append(0)

else:
    rospy.logwarn('Incorrect initial state specified, defaulting to scattered configuration')
    # belowBlock all zeros

# END INITIAL PREP
#######################################################################################
# BEGIN FUNCTION DEFINITIONS

def updateState(action, target):



def handle_move_robot(request):
    
    success = updateState(request.action, request.target)
    return MoveRobotResponse(success)


def sim_master():  
    
    pubState = rospy.Publisher('state', State, queue_size=10)
    rospy.init_node('sim_master')
    rospy.Service('move_robot', MoveRobot, handle_move_robot)

    rate = rospy.Rate(1) # 1 Hz
    
    rospy.loginfo("Prep work complete, sim_master is online! [%s]" % rospy.get_time())

    # Main loop here
    while not rospy.is_shutdown():
        pubState.publish(belowBlock=,hasObj=,openGrip=) 
        rate.sleep()

if __name__ == '__main__':
    try:
        sim_master()
    except rospy.ROSInterruptException:
        pass
