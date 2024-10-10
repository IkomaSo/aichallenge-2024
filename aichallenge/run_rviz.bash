source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

ros2 launch aichallenge_submit_launch rviz.launch.xml 