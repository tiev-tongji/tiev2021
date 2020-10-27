#gnome-terminal -x bash -c "pwd;exec bash;"
gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/NtripClientPy/;./Run-ntrip-tiev-changshu-verbose.sh;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/IMU_Receiver_linux_zcm/build;./IMU_Receiver;exec bash;"

gnome-terminal -x bash -c "source activate second;cd /home/autolab/tiev2019/src/modules/MultiBeamLaser/build;./MultibeamLaser;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/Sick/build;./Sick;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/PerceptionFusion/PerceptionFusion/build;./perceptionfusion_for_competition;exec bash;"

#gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/acc_back/build;./acc;exec bash;"

#gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/planner/src/build/start_up;./planner;exec bash;"

#gnome-terminal -x bash -c "ssh ubuntu@192.168.0.110;exec bash;"
