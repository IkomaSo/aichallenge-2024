#!/bin/bash

mode=${1}

case "${mode}" in
"awsim")
    opts="simulation:=true use_sim_time:=true run_rviz:=true wheel_base:=1.087"
    ;;
"vehicle")
    opts="simulation:=false use_sim_time:=false run_rviz:=false wheel_base:=1.28"
    ;;
"rosbag")
    opts="simulation:=false use_sim_time:=true run_rviz:=true"
    ;;
*)
    echo "invalid argument (use 'awsim' or 'vehicle' or 'rosbag')"
    exit 1
    ;;
esac

trajectory=${2}

traj_path="/aichallenge/workspace/install/sanae_planner/share/sanae_planner/trajectory/${trajectory}.csv"
if [ ! -e ${traj_path} ]; then
    echo "traj_path: ${traj_path}"
    echo "invalid trajectory"
    exit 1
fi

traj_opts="trajectory:=${trajectory}"

opts="${opts} wheel_base:=${3}"

controller=${4}
preset=${5}

if [ "${controller}" = "smc" ]; then
    opts="${opts} controller:=smc"
elif [ "${controller}" = "pid" ]; then
    opts="${opts} controller:=pid"
else
    echo "invalid controller (use 'smc' or 'pid')"
    exit 1
fi

config_path="/aichallenge/workspace/install/sanae_control/share/sanae_control/config/${preset}.yaml"
if [ ! -e ${config_path} ]; then
    echo "config_path: ${config_path}"
    echo "invalid preset"
    exit 1
fi

preset_opts="controller_param:=${preset}"

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo

# shellcheck disable=SC2086
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml ${opts} ${traj_opts} ${preset_opts}