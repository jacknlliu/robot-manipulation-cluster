#!/bin/bash

USER_BASH_RC="/home/ros/.bashrc"


function stop-background-apps() {
  if [[ $(jobs -p|wc -l) -ge 1 ]]; then
    kill -2 $(jobs -p) >/dev/null 2>&1
  fi
}

if [[ $# -ne 2 ]]; then
  echo "Usages: $0 ROS_MASTER_URI MYSELF_CONTAINER_IP"
  echo "for example, $0 "http://10.2.2.2:11311" "10.2.2.2""
  exit 1
else
  # start rosmaster
  export ROS_MASTER_URI=$1
  export ROS_IP=$2

  # add find string ROS_MASTER_URI in /home/ros/.bashrc,
  # if exists delete and add new, if not, add new.
  echo "export ROS_MASTER_URI=$1" >> $USER_BASH_RC
  echo "export ROS_IP=$2" >> $USER_BASH_RC
  source $USER_BASH_RC

  # wait for master
  while [[ ! $(rostopic list 2>/dev/null) ]]; do
    sleep 1
    printf " waiting for ros master \r"
  done

  echo "connected to ros master"

  # add camera device recognize

  # start camera 0
  # set re-enter and device access
  if [[ -e /dev/video0 ]]; then
    sudo chmod a+rwx /dev/video0

    # add retry open camera
    roslaunch video_stream_opencv camera.launch camera_name:=camera0 video_stream_provider:=0 &
    camera0_pid=$!
  else
    echo "camera0  not found!"
  fi


  # start camera 1
  if [[ -e /dev/video1 ]]; then
    sudo chmod a+rwx /dev/video1

    # add retry open camera
    roslaunch video_stream_opencv camera.launch camera_name:=camera1  video_stream_provider:=1 &
    camera1_pid=$!
  else
    echo "camera1  not found!"
  fi

  trap "stop-background-apps" 2 9
  wait $camera0_pid $camera1_pid
fi
