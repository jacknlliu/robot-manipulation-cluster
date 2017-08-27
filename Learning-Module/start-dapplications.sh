#!/bin/bash

USER_BASH_RC="/home/ros/.bashrc"

if [[ $# -ne 4 ]]; then
  echo "Usages: start-dapplications.sh ROS_MASTER_URI MYSELF_CONTAINER_IP"
  echo "for example, start-dapplications.sh "http://10.2.2.2:11311" "10.2.2.2""
  exit 1
else
  # start rosmaster
  export ROS_MASTER_URI=$1
  export ROS_IP=$2

  # add find string ROS_MASTER_URI in /home/ros/.bashrc,
  # if exists delete and add new, if not, add new.
  echo "export ROS_MASTER_URI=$1" >> $USER_BASH_RC
  echo "export ROS_IP=$2" >> $USER_BASH_RC
  echo "export ROBOT_IP_ADDR=$3" >> $USER_BASH_RC
  echo "export LOCAL_HOST_IP=$4" >> $USER_BASH_RC

  # set PS1
  echo "export PS1='[\u@\h: \W]\$ '" >> $USER_BASH_RC

  source $USER_BASH_RC

  # just for learning module because of python3 environment with ros python 2.7
  source /opt/ros/kinetic/setup.bash

  # wait for master
  while [[ ! $(rostopic list 2>/dev/null) ]]; do
    sleep 1
    printf " waiting for ros master \r"
  done

  echo "connected to ros master"

  # run bash with docker run option '-t', avoid the container exit.
  bash -l
  exit 0
fi
