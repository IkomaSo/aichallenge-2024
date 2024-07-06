#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM

# shellcheck disable=SC1091
echo "Start AWSIM"
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
$AWSIM_DIRECTORY/AWSIM.x86_64 >/dev/null &
PID_AWSIM=$!
sleep 20

echo "Start Rviz"
ros2 launch aichallenge_submit_launch only_localization.launch.xml >/dev/null 2>&1

echo "Stop AWSIM"
kill $PID_AWSIM
wait $PID_AWSIM 