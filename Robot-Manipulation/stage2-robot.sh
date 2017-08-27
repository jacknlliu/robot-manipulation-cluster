#!/bin/bash

# this should be customized by user
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 start|stop ws_path" >&2
  exit 1
elif [[ $1 = "start" ]]; then
  if [[ $# -ne 2 ]]; then
    echo "Usage: $0 start ws_path" >&2
    exit 1
  else
    UR5_WS_PATH=$2
  fi
fi

function start-robot() {
  if [[ -n $ROS_IP && -n $ROS_MASTER_URI ]]; then
    cd $UR5_WS_PATH
    source $UR5_WS_PATH/devel/setup.bash
    echo "start connecting UR5 robot..."
    # set re-enter for command
    # set stop command
    roslaunch --pid=/tmp/ur5_bringup.pid ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=$ROBOT_IP_ADDR reverse_ip:=$LOCAL_HOST_IP &

  else
    echo "ROS_IP and ROS_MASTER_URI not set correctly!"
    return 1
  fi
}

function stop-robot() {
  if [[ -e /tmp/ur5_bringup.pid ]]; then
    kill -INT `cat /tmp/ur5_bringup.pid`
  fi
}


case $1 in
  start )
    start-robot
    ;;
  stop )
    stop-robot
    ;;
esac
