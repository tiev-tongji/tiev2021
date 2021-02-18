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

#check this ubuntu whether has tmux
type tmux
if [ $? != 0 ]
then
	sudo apt update
	sudo apt install tmux -y
fi

#check the tiev session whether is existing
tmux has-session -t tiev
if [ $? != 0 ]
then
	tmux new-session -s tiev -n tiev2020 -d
	tmux split-window -v -t tiev:tiev2020.0
	tmux split-window -h -t tiev:tiev2020.0
	tmux split-window -h -t tiev:tiev2020.2
	#start visualization
	tmux send-keys -t tiev:tiev2020.0 'cd ~/tiev2020-code/src/modules/planner/build/' C-m
	tmux send-keys -t tiev:tiev2020.0 './src/visualization/visualization' C-m
	#start planner
	tmux send-keys -t tiev:tiev2020.1 'cd ~/tiev2020-code/src/modules/planner/build/' C-m
	tmux send-keys -t tiev:tiev2020.1 './src/planner' C-m
	#start trajectory controller
	tmux send-keys -t tiev:tiev2020.2 'cd ~/tiev2020-code/src/modules/Trajectory_Controller/build' C-m
	tmux send-keys -t tiev:tiev2020.2 './trajectory_controller' C-m
	##start vehicle controller
	#tmux send-keys -t tiev:tiev2020.3 'cd ~/tiev2020-code/src/modules/planner/build/src' C-m
	#tmux send-keys -t tiev:tiev2020.3 './planner' C-m
fi
tmux attach -t tiev
