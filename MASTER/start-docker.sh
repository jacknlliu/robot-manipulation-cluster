#!/bin/bash

LOCAL_HOST_IP="192.168.1.123"

PARTNER_IP_ADDRs="192.168.1.122"

MYSELF_CONTAINER_IP="10.2.2.2"

ROS_MASTER_URI="http://10.2.2.2:11311"

ROBOT_IP_ADDR="192.168.1.125"

DATA_SHARE_DIR="/home/tree/Workspace"
GAZEBO_MODEL_DIR="/home/tree/Workspace/gazebo_models"

CONTAINER_INIT_DIR=$(readlink -f "$0" |xargs dirname)

WEAVE_IP_RANGE="10.2.2.0/24"

CONTAINER_NAME="ros_kinetic_weave_master"

# only for which has local weave peers
LOCAL_WEAVE_NICKNAME="local-weave-peers"

if [[ -f /etc/fedora-release ]]; then
  sudo iptables -D FORWARD -j REJECT --reject-with icmp-host-prohibited >/dev/null 2>&1
fi


function stop_weave() {
  # stop weave network, and restart weave network
  weave stop  > /dev/null  2>&1
  weave reset --force  > /dev/null 2>&1

  end_loop_time=$((SECONDS+30))
  docker inspect -f {{.State.Status}} weave > /dev/null 2>&1
  while [[ $? -eq 0  && $SECONDS -lt $end_loop_time ]]; do
    docker inspect -f {{.State.Status}} weave > /dev/null 2>&1
    sleep 2
    printf " waiting for previous weave exit...\r"
  done

  # for fixed known issue
  if [[ $(docker inspect -f {{.State.Status}} weave 2>/dev/null) = "dead" ]]; then
     sudo pkill -9 fwupd >/dev/null 2>&1
     docker rm -f weave >/dev/null 2>&1
     if [[ $? -eq 0 ]]; then
        echo "removed previous weave successfully."
     fi
  fi
}


function  test_partner_ip() {
  for partner_ip in $@; do
    ping -c1 $partner_ip >/dev/null 2>&1
    if [[  $? -ne 0 ]]; then
      echo "$partner_ip is not valid" >&2
      return 1
    fi
  done

  return 0
}

function restart_local_weave() {

  stop_weave

  # start weave network
 test_partner_ip $PARTNER_IP_ADDRs
 if [[ $? -ne 0 ]]; then
   exit 1
 fi


  echo "start weave network..."
  weave launch --ipalloc-range $WEAVE_IP_RANGE --nickname "$LOCAL_WEAVE_NICKNAME" $PARTNER_IP_ADDRs
  if [[ $? -eq 0 ]]; then
    echo "start weave network successfully!"
  else
    echo "ERROR: start weave network failed!"
    exit 1
  fi
}

# TODO: refractor code
# function restart_weave() {
# }

# stop docker container
if [[ -n $(docker ps -a -q  -f  "name=$CONTAINER_NAME") ]]; then
  container_name_state=$(docker inspect -f "{{if eq \"/$CONTAINER_NAME\" .Name }}{{.Name}}{{ end }}" $(docker ps -a -q  -f  "name=$CONTAINER_NAME"))
  if [[ $container_name_state = "/"$CONTAINER_NAME ]]; then
    docker stop $CONTAINER_NAME > /dev/null 2>&1
    docker rm $CONTAINER_NAME > /dev/null 2>&1
  fi
fi


# start weave net
if [[ $# -eq 1 ]]; then
  if [[ $1 = "--with-local-peers" ]]; then
    echo "start with local peers..."

    # check weave status, if no weave network, start it
    if [[ $(weave status | wc -l) -gt 20 &&  $(weave report -f '{{ .Router.NickName}}' 2>/dev/null) = "$LOCAL_WEAVE_NICKNAME" ]]; then
      echo "local weave network is working"
    else
      restart_local_weave
    fi
  else
    echo "Usage: $0 [--with-local-peers] "
    echo "      --with-local-peers: start with local peers"
    exit 1
  fi
elif [[ $# -eq 0 ]]; then
  echo "Usage: if you have local peers, you should run $0 --with-local-peers."
  echo "This command run as no local peers, and it will remove local weave net."

  stop_weave

  # start weave network
  test_partner_ip $PARTNER_IP_ADDRs
  if [[ $? -ne 0 ]]; then
    exit 1
  fi

  echo "start weave network..."
  weave launch --ipalloc-range $WEAVE_IP_RANGE  $PARTNER_IP_ADDRs
  if [[ $? -eq 0 ]]; then
    echo "start weave network successfully!"
  else
    echo "ERROR: start weave network failed!"
    exit 1
  fi
fi


if [[ -f $CONTAINER_INIT_DIR/start-dapplications.sh ]]; then
  # start docker container
  docker run --privileged -d -t --rm --net weave --ip $MYSELF_CONTAINER_IP \
         --security-opt label=disable   --security-opt seccomp=unconfined \
         --env="DISPLAY" --env QT_X11_NO_MITSHM=1 \
         --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
         --volume="$DATA_SHARE_DIR:/data:rw" \
         --volume="$GAZEBO_MODEL_DIR:/home/ros/.gazebo/models:rw" \
         --volume="$CONTAINER_INIT_DIR:/home/ros/container-scripts:rw" \
         --name=$CONTAINER_NAME   \
         jacknlliu/ros:kinetic-ide-init /home/ros/container-scripts/start-dapplications.sh $ROS_MASTER_URI $MYSELF_CONTAINER_IP $ROBOT_IP_ADDR $LOCAL_HOST_IP
else
  echo "ERROR: no start-dapplications.sh found!"
  exit 1
fi
