#gnome-terminal -x bash -c "pwd;exec bash;"
#gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/IMU_Receiver_linux_lcm/build;./IMU_Receiver;exec bash;"

#gnome-terminal -x bash -c "source activate second;cd /home/autolab/tiev2019/src/modules/MultiBeamLaser/build;./MultibeamLaser;exec bash;"
#
#gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/Sick/build;./Sick;exec bash;"

#gnome-terminal -x bash -c "cd /home/autolab/Desktop/TiEV/src/modules/MultiBeamLaser/build;./MultibeamLaser;exec bash;"

#gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/PerceptionFusion/PerceptionFusion/build;./perceptionfusion;exec bash;"

#gnome-terminal -x bash -c "cd /home/autolab/tiev2019/src/modules/acc_back/build;./acc;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/planner/build/src;exec bash"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/Trajectory_Controller/build;./trajectory_controller;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/vision/RoadmarkingDect/;python3 road_marking_dect_online.py;exec bash;"

gnome-terminal -x bash -c "cd /home/autolab/tiev2020-code/src/modules/SurroundVisionSlotDetection/;python3 main.py;exec bash;"

gnome-terminal -x bash -c "ssh autolab@192.168.222.110;exec bash;"
