####how to run:
- #####build
	+ mkdir build
	+ cd build
	+ cmake ..
	+ make -j
- #####run
	+ source activate second
	+ ./MultibeamLaser

####environment dependence:
- boost 1.58
- opencv >= 3.2
- eigen3
- zcm
- anaconda python

####files structure:
- #####calibration input files: 
	- db.xml and ID89.cal
- #####param files: 
	- **param.json**
		+ sourcePointcloudEnable : whether show the source pointcloud in grid map, default false;
		+ is_mot_enable : whether show the dynamic obstacles in grid map, default true;
		+ overpass_height : not important, default 1.5;
		+ z_resolution : decide which point belongs to obstacle, the larger the value, the more obstacles but the more noise, default 0.3;
		+ z_obstacle_height : decide which cell in grid belongs to obstacle, the smaller the value, the more obstacles, default 0.09;
		+ yaw, pitch, roll, dx, dy, dz: lidar extern parameter refer to front axis of car;
- #####src/global:
	+ grid definition and some global funtion, it is not necessary read in depth.
- #####src/tiny:
	+ calibration for every point received from lidar, it is not necessary read in depth.
- #####src/perception(important):
	+ **MultiBeamLaser.cpp** main entry, read json parametry, start 5 thread,     
		+ zcm_udpm_receiver_thread: receive udpm msg by zcm;
    	+ zcm_ipc_receiver_thread: receive ipc msg by zcm;
    	+ lidar_recieve_thread: receive lidar packet by ethernet upm;
    	+ calibration_HDL64_thread: calibration packet pointcloud and get one frame;
    	+ object_tracking_thread: get latest frame and latest NAVINFO, generate lasermap and objectlists;
    	+ localization_indoor_thread: publish pointcloud for indoor locaization;
	+ **integration.cc**
		+ build z_grid, second velo points to velodyne.cc
		+ start kalman tracker 
		+ publish dynamic objectlists 
	+ **velodyne.cc**
		+ define which point belong to obstacle
		+ define which cell belong to obstacle
		+ generate lasermap and publish
	+ **second_receiver.cpp**
		+ pass the latest velodyne pointclouds to second net
		+ return detection result by calling python script
	+ **kalman_multitracker.cpp**
		+ match the trackers and the measurements by scores matrix 
		+ the tracker start linear kalman filter prediction and update step if matched, 
		+ the tracker start linear kalman filter prediction step if not matched, 
		+ the measurements will initialize as trackers if measurements left.
	+ **linear_kalman_filer.cpp**
		+ kalman predict and update step
	+ **tracked_obstacle.cpp**
		+ update the tracker state by matcked measurement, and mark which tracker is dynamic
	+ **obstacle.cpp**
		+ draw the obstacle bounding box in grid map
	+ **data_association.cpp**
		+ bipartiate graph data association
	+ **successive_shortest_path.cpp**
		+ how to select the best match
+ #####include/perception(important):
	+ **perception.h**
		+ myZcm class to receice msg and publish msg
		+ MyVisualization for grid map show
		+ some global variable

