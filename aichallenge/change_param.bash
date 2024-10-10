#!/bin/bash

while : ;do
  clear
  ros2 param get $1 $2
  read -p "$3: " new_value
  ros2 param set $1 $2 $new_value
done