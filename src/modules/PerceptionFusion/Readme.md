#PerceptionFusion——2019

- 功能介绍

  调用Cartographer库，实现3DoF建图、定位功能。根据任务分成两种模式：

  - above_ground

    接收LaserMap（laserscan与RTK），进行occupancy grid拼接建图。根据RTK位姿，读取历史地图，并融合多种雷达数据源，发送fusionmap。

  - under_ground_mapping&under_ground_location

    接收三维点云，进行cartographer2D slam建图、定位。分为建图模式与纯定位模型。

    建图原理详见[cartographer](https://github.com/googlecartographer/cartographer)。

- 编译方法

  - 环境安装：

    - cartographer环境

      ```
      sudo apt-get install -y google-mock libboost-all-dev  libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libprotobuf-dev  libsuitesparse-dev libwebp-dev ninja-build protobuf-compiler python-sphinx  ros-indigo-tf2-eigen libatlas-base-dev libsuitesparse-dev liblapack-dev
      ```

    - Opencv
    - zcm
    - eigen

  - 编译方法

    ```
    sh PerceptionFusion/make.sh
    ```

- 参数设置

  分为两部分:

  - 地图路径与模式改变由PerceptionFusion/parameters.txt控制

    ```
    UpdateMapFlag=1对应接受lasermap数据进行地图更新
    LoadMapFlag=1对应读图模式
    map_filename对应读取的地图文件
    SaveMapFlag=1对应建图模式
    debug_show_map=1显示当前地图（需要保存的时候需要设为0 因为opencv的冲突问题）
    mode=1/2/3
    #three mode :
    #1. above_ground
    #2. under_ground_mapping
    #3. under_ground_location
    ```

    - above_ground建图与更新

      开启MultibeamLaser和RTK模块后,启动PerceptionFusion,接收到LaserMap开始建图。

      建图完成后,在窗口内按"s"键退出建图模式并在`save_path`下保存地图。

      ```
      mode=1
      UpdateMapFlag=1
      LoadMapFlag=0/1（取决于初次建图还是更新建图）
      map_filename对应读取的地图文件
      SaveMapFlag=1
      debug_show_map=0
      ```

    - above_ground只读图（比赛时只读取历史地图并融合）

      开启MultibeamLaser,RTK,Lux,Sick模块后启动PerceptionFusion。将融合并发送FusionMap。

      ```
      mode=1
      UpdateMapFlag=0
      LoadMapFlag=1
      map_filename对应读取的地图文件
      其余随意
      ```

    - under_ground_mapping

      接收点云进行slam建图。

      ```
      mode=2
      LoadMapFlag=0/1（取决于初次建图还是更新建图）
      map_filename对应读取的地图文件
      SaveMapFlag=1
      debug_show_map=0
      其余随意
      ```

    - under_ground_location

      对应cartographer的pure_localization功能。接收点云进行局部建图并用于全局定位。

      ```
      mode=3
      LoadMapFlag=1
      map_filename对应读取的地图文件
      debug_show_map=0
      其余随意
      ```

  - cartographer参数修改位于Cartographer/configuration_files下的lua参数文件，可参考[catographer document](https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html)与[CSDN介绍](https://blog.csdn.net/tiancailx/article/details/90757522)

    - above_ground

      ```
      其中主要参数有：
      backpack_2d.lua下
      	TRAJECTORY_BUILDER_2D.scans_per_accumulation控制submap的生成速率
      trajectory_builder_2d.lua下
      	max_range     可过滤较远的点
      		num_range_data  控制submap的生成速率（同上）
        		hit_probability  miss_probability   判断为障碍物的概率 根据点云质量调整 如果较差便调低
      ```

    - under_ground_mapping&under_ground_location

      ```
      trajectory_builder_2d.lua下
      	  min_range = 2.0,--0
            max_range = 30.,--30
            min_z = -1.6,
            max_z = -1.1,
      sparse_pose_graph.lua控制回环参数
      ```

