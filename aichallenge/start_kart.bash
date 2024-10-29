#!/bin/bash

ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{ 
  header: {
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {
        x: 89633.29,
        y: 43127.57,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.8778,
        w: 0.4788
      }
    }
  }
}" >/dev/null

sleep 1

ros2 service call /control/control_mode_request autoware_auto_vehicle_msgs/srv/ControlModeCommand '{mode: 1}' >/dev/null
