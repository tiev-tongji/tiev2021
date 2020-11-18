#gnome-terminal -x bash -c "pwd;exec bash;"
gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/NtripClientPy/;./Run-ntrip-tiev-changshu-verbose.sh;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/IMU_Receiver_linux_zcm/build;./IMU_Receiver;exec bash;"

gnome-terminal -x bash -c "source activate second;cd /home/autolab/tiev2020-code/src/modules/MultiBeamLaser/build;./MultibeamLaser;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/Sick/build;./Sick;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/PerceptionFusion/PerceptionFusion/build;./perceptionfusion;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/planner/build/src;./visualization/visualization;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/Trajectory_Controller/build;./trajectory_controller;exec bash;"

gnome-terminal -x bash -c "ssh autolab@192.168.222.110;exec bash;"
