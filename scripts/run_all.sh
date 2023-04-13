#!/bin/bash
###################################
#TiEV start script
#Panel index:name
#0:visualization
#1:planner
#2:trajectory controller
#3:vehicle controller
#4:
#5:
#6:
#7:
#8:
#9:
#10:
##################################

#configure tmux
#echo "set-option -g mouse on" | tee ~/.tmux.conf

#check the tiev session whether is existing
tmux has-session -t tiev
if [ $? != 0 ]
then
	tmux new -s tiev -n tiev2021 -d
	tmux split-window -v -t tiev:tiev2021.0
	tmux split-window -v -t tiev:tiev2021.1
	tmux split-window -v -t tiev:tiev2021.2
	#tmux split-window -h -t tiev:tiev2021.3
	#start visualization
	tmux send-keys -t tiev:tiev2021.0 'cd ~/tiev/scripts/' C-m
	tmux send-keys -t tiev:tiev2021.0 './planner.sh' C-m
	#start planner
	tmux send-keys -t tiev:tiev2021.1 'cd ~/tiev/scripts/' C-m
	tmux send-keys -t tiev:tiev2021.1 './trajectory_controller.sh' C-m
	#start trajectory controller
	tmux send-keys -t tiev:tiev2021.2 'cd ~/tiev/scripts/' C-m
	#start trajectory controller
	tmux send-keys -t tiev:tiev2021.3 'cd ~/tiev/scripts/' C-m
	tmux send-keys -t tiev:tiev2021.3 'tmux select-pane -U' C-m
	#start can control ZLG
	#tmux send-keys -t tiev:tiev2021.3 'cd ~/tiev/scripts/' C-m
	#tmux send-keys -t tiev:tiev2021.3 './can_control.sh' C-m
	#start ControlCenterZLG_sim
	#tmux send-keys -t tiev:tiev2021.3 'cd ~/tiev/scripts/' C-m
	#tmux send-keys -t tiev:tiev2021.3 './controlcenterzlg_sim.sh' C-m
	##start vehicle controller
	#tmux send-keys -t tiev:tiev2021.3 'cd ~/tiev2021/src/modules/planner/build/src' C-m
	#tmux send-keys -t tiev:tiev2021.3 './planner' C-m
fi
tmux attach -t tiev
