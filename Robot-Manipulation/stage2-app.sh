#!/bin/bash

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 start|stop"
  exit 1
fi

LEFT_CAMERA_TOPIC="/camera1/image_raw"
RIGHT_CAMERA_TOPIC="/camera0/image_raw"

function start-cameras() {
  # after image node start
  echo "This shell script must run after image capture node startup!"
  echo "This script should run by normal user!"

  if [[ -n $ROS_IP && -n $ROS_MASTER_URI ]]; then
    if [[ -n $(rostopic list |grep -w "$LEFT_CAMERA_TOPIC") ]]; then
      if [[ -n $(pgrep  -f  "/image_view/image_view image:=$LEFT_CAMERA_TOPIC") ]]; then
        echo "Another image_view for left camera topic $LEFT_CAMERA_TOPIC is working,"
        echo "we will keep no change for it."
      else
        # show image for left camera
        rosrun image_view image_view  image:=$LEFT_CAMERA_TOPIC &

        # start find_object_2d node
        rosrun find_object_2d find_object_2d image:=$LEFT_CAMERA_TOPIC &
      fi
    else
      echo "Waring: Left camera not ready!"
    fi

    if [[ -n $(rostopic list |grep -w "$RIGHT_CAMERA_TOPIC") ]]; then
      if [[ -n $(pgrep  -f  "/image_view/image_view image:=$RIGHT_CAMERA_TOPIC") ]]; then
        echo "Another image_view for right camera topic $RIGHT_CAMERA_TOPIC is working,"
        echo "we will keep no change for it."
      else
        rosrun image_view image_view  image:=$RIGHT_CAMERA_TOPIC &
        rosrun find_object_2d find_object_2d image:=$RIGHT_CAMERA_TOPIC &
      fi
    else
      echo "Waring: Right camera not ready!"
    fi

  else
    echo "Waring: ROS_MASTER_URI and ROS_IP not be set correctly!"
    echo "Nothing done!"
  fi
}


function stop-cameras() {
  # after image node start
  echo "This shell script must run after image capture node startup!"
  echo "This script should run by normal user!"

  LEFT_CAMERA_TOPIC="/camera1/image_raw"
  RIGHT_CAMERA_TOPIC="/camera0/image_raw"

  if [[ -n $ROS_IP && -n $ROS_MASTER_URI ]]; then

     # stop left camera
      if [[ -n $(pgrep  -f  "/image_view/image_view image:=$LEFT_CAMERA_TOPIC") ]]; then
          kill -9  $(pgrep  -f  "/image_view/image_view image:=$LEFT_CAMERA_TOPIC")
      else
          echo "Left camera not open. Nothing to be done."
      fi

      if [[ -n $(pgrep  -f  "find_object_2d image:=$LEFT_CAMERA_TOPIC") ]]; then
        kill -9 $(pgrep  -f  "find_object_2d image:=$LEFT_CAMERA_TOPIC")
      else
        echo "Left camera find_object_2d not open. Nothing to be done. "
      fi

      # stop right camera
      if [[ -n $(pgrep  -f  "/image_view/image_view image:=$RIGHT_CAMERA_TOPIC") ]]; then
         kill -9  $(pgrep  -f  "/image_view/image_view image:=$RIGHT_CAMERA_TOPIC")
      else
        echo "Right camera not open. Nothing to be done."
      fi

      if [[ -n $(pgrep  -f  "find_object_2d image:=$RIGHT_CAMERA_TOPIC") ]]; then
        kill -9 $(pgrep  -f "find_object_2d image:=$RIGHT_CAMERA_TOPIC")
      else
        echo "Left camera find_object_2d not open. Nothing to be done. "
      fi


  else
    echo "Waring: ROS_MASTER_URI and ROS_IP not be set correctly!"
    echo "Nothing done!"
  fi
}


case $1 in
  start )
    start-cameras
    ;;
  stop )
    stop-cameras
    ;;
esac
