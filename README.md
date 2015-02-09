# MAE-4750-HW1
Robotic Manipulation HW 1
BY: Anthony, Andy, Alex

INSTRUCTIONS:

Launch full system with roslaunch:

*NOTE: Unfortunately we can't seem to get roslaunch to properly start our nodes.
It complains that the nodes do not exist in the package.

$ roslaunch am_zl_av_4750_hw1 hw1.launch



Launch system using rosparam and rosrun:

$ rosparam set num_blocks 10
$ rosparam set configuration 'scattered'
$ rosrun am_zl_av_4750_hw1 sim_master.py
[new terminal]
$ rosrun am_zl_av_4750_hw1 controller.py
[new terminal]
$ rostopic pub /command std_msgs/String 'stacked_ascending'
