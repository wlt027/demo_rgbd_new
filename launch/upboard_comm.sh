#!/bin/zsh

### according to this webpage http://evakasch.github.io/2016/04/13/ros-comm/

export ROS_MASTER_URI=http://robot:11311
export ROS_IP="192.168.100.3"
export ROS_HOSTNAME="davidz-T430"

# rosrun rviz rviz -d "vio_laser.rviz"
roslaunch show_result.launch
# roslaunch show_rviz.launch
