#!/bin/bash

AIC_CD_CMD="cd /aichallenge"
SOURCE_CMD="source workspace/install/setup.bash"

TRAJ_NODE_NAME="/planning/scenario_planning/trajectory_optimizer"
CONTROL_NODE_NAME="/simple_pd_controller"

# mouse setup
set-option -g mouse on
bind-key -n WheelUpPane if-shell -F -t = "#{mouse_any_flag}" "send-keys -M" "if -Ft= '#{pane_in_mode}' 'send-keys -M' 'select-pane -t=; copy-mode -e; send-keys -M'"
bind-key -n WheelDownPane select-pane -t= \; send-keys -M

set-window-option -g mode-keys vi
bind-key -T copy-mode-vi v send -X begin-selection
bind-key -T copy-mode-vi y send -X copy-pipe-and-cancel "xclip -sel clip -i"

# split-vertical
# 1. 初期のペインで左右に分割
select-pane -t 0         
split-window -h -p 50     

# 2. 左側のペインを3つに分割
select-pane -t 0         
split-window -v -p 50
select-pane -t 0
split-window -v -p 50
select-pane -t 2
split-window -v -p 50 

# 3. 右側のペインを選択して上下に 4 回分割
select-pane -t 4 
split-window -v -p 50
select-pane -t 4
split-window -v -p 50
select-pane -t 6
split-window -v -p 50 

select-pane -t 0
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./change_param.bash $TRAJ_NODE_NAME speed_km_h speed" C-m

select-pane -t 1
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./change_param.bash $TRAJ_NODE_NAME course_margin margin" C-m

select-pane -t 2
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./change_param.bash $TRAJ_NODE_NAME curve_weight curve" C-m

select-pane -t 3
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./emergency.py" C-m

select-pane -t 4
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./change_param.bash $CONTROL_NODE_NAME steering_angle_proportional_gain steer_p" C-m

select-pane -t 5
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./change_param.bash $CONTROL_NODE_NAME steering_angle_derivative_gain steer_d" C-m

select-pane -t 6
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./change_param.bash $CONTROL_NODE_NAME lookahead_gain look_gain" C-m

select-pane -t 7
send-keys "set -x" C-m
send-keys "$AIC_WORKSPACE_CD_CMD" C-m
send-keys "$SOURCE_CMD" C-m
send-keys "./change_param.bash $CONTROL_NODE_NAME lookahead_min_distance look_min" C-m

