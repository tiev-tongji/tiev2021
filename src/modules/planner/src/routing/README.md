## 一次规划环境配置

### server
+ install
  ```shell
  # server info
  ssh ubuntu@106.75.246.176

  # 添加国内镜像仓库
  sudo sh -c 'echo "deb http://mirrors.ustc.edu.cn/postgresql/repos/apt $(lsb_release -cs)-pgdg main" > /etc/apt/sources.list.d/pgdg.list'
  # 导入签名
  wget --quiet -O - http://mirrors.ustc.edu.cn/postgresql/repos/apt/ACCC4CF8.asc | sudo apt-key add -
  # 更新列表
  sudo apt-get update
  # 安装制定版本的PostgreSQL.
  sudo apt-get install postgresql-12 postgresql-contrib
  # Install other related packages
  sudo apt-get install pgadmin4 postgis postgresql-12-pgrouting
  ```

### server configuration

+ postgresql 设置密码
  ```shell
  sudo -u postgres -i
  # 进入postgres shell
  psql
  \password
  (please set password: routing2020)
  # 关闭自动提交
  \set AUTOCOMMIT off
  \q
  ```

+ postgresql 配置
  ```shell
  # 如果数据库在本地
  # hostname:port:database_name:username:password
  vim ./.pgpass
  127.0.0.1:5432:gis:postgres:postgres

  # 如果数据库在服务器,则在远端进行如下操作
  vim /etc/postgresql/12/main/postgresql.conf
  # listen_addresses = ‘localhost‘ 的注释去掉并改为 :
  isten_addresses = ‘*‘

  # 修改可访问用户ip
  vim /etc/postgresql/12/main/pg_hba.conf
  # 文件末尾加入
  host all all 0.0.0.0 0.0.0.0 trust
  # 表示允许全部ip免密码访问(有安全风险) trust换成md5表示需要密码.
  # 重启数据库
  sudo /etc/init.d/postgresql restart
  ```
+ 使用pgadmin4 加载postgresql的拓展
  ```
  CREATEDB -E -UTF8 [db_name]
  CREATE EXTENSION hstore;
  CREATE EXTENSION fuzzystrmatch;
  CREATE EXTENSION postgis_topology;
  CREATE EXTENSION postgis_tiger_geocoder;
  # 看起来只有最后一个是有必要加载的拓展
  CREATE EXTENSION pgrouting CASCADE;
  ```
+ 使用命令行或pgadmin4加载sql脚本函数
  ```shell
  # 远程操作加入 -h your_host_ip
  # psql -U username -d myDataBase -f scripts.sql
  sudu -u postgres -i
  psql -U postgres -d changshu -f shortestPath.sql
  psql -U postgres -d changshu -f pgr_pointoffset.sql
  psql -U postgres -d changshu -f pgr_updateTopoMap.sql
  psql -U postgres -d changshu -f outputPointPath.sql
  ```
+ NOTE: postgrtes12 默认端口不一定是 5432
  ``` shell
  vim etc/postgresql/12/main/postgresql.config
  ```
+ 备忘:pgadmin pwd: postgres
+ 启动postgresql服务 (数据库在本地时, 打开pgadmin4后默认开启打开服务)
  ```
  /etc/init.d/postgresql start
  /etc/init.d/postgresql stop
  /etc/init.d/postgresql restart
  ```
### 开发端配置
+ install
  ```
  # libpqxx 安装
  # 可能的替代(ubutnu18只支持liboqxx到version4, 经测试可用)
  sudo apt-get install libpqxx-dev

  # 源码安装方法
  git clone https://github.com/jtv/libpqxx.git
  git checkout 6.4.7  #最后一个不需要c++17标准的版本
  mkdir build
  cd build
  camke .. 
  make
  make install
  ```
+ **usage**
  开发时```#include <pqxx/pqxx>```即可, 本模块build后会在mapper/lib生成librouting.so动态库, 请在代码中```#include Routing.h" ```, 并在cmake中链接该动态库.

### 导入地图文件和属性文件
+ 导入拓扑地图(用于寻径)
  ```
  shp2pgsql -s 4326 -d your_path/ topo_map | psql -h 106.75.246.176 -p 5432 -U postgres -d changshu2020
  ```
  topo_map, 依次在pgadmin query tool中执行即可
  ```sql
  ALTER TABLE topo_map 
  ADD COLUMN source integer,
  ADD COLUMN target integer,
  ADD COLUMN length double precision,
  ADD COLUMN cost double precision,
  ADD COLUMN geog geography;
  ALTER TABLE topo_map RENAME COLUMN geom TO the_geom;
  -- geometry to geography
  UPDATE topo_map SET geog =  Geography(the_geom);
  UPDATE topo_map SET length = ST_Length(geog);
  -- todo set cost = cost_time (s)
  UPDATE topo_map SET cost = length

  SELECT pgr_createTopology('topo_map', 0.000001, 'the_geom', 'gid');
  ALTER TABLE topo_map ALTER COLUMN the_geom TYPE geometry(linestring,4326) USING ST_GeometryN(the_geom, 1);
  ```

+ 创建语义地图点表, 并上传.
  ```sql
  drop table if exists all_points;
  CREATE TABLE all_points(
          p_id bigint,
          utmX real,
          utmY real,
          heading real,
          curv real,
          mode bigint,
          speed_mode bigint,
          event_mode bigint,
          opposite_side_Mode bigint,
          lane_num bigint,
          lane_seq bigint,
          lane_width real,
          geom geometry(Point, 4326)
        );
        
  drop table if exists stop_points;
  CREATE TABLE stop_points(
          p_id bigint,
          utmX real,
          utmY real,
          heading real,
          curv real,
          mode bigint,
          speed_mode bigint,
          event_mode bigint,
          opposite_side_Mode bigint,
          lane_num bigint,
          lane_seq bigint,
          lane_width real,
          geom geometry(Point, 4326)
        );
  ```
  以上见import_map.sql
+ 如何获取规划结果数据
  ```
  psql -h 10.60.144.32 -p 5432 -U postgres -d changshu -w -c "\copy result_path_points to '/tmp/shortest_road.csv' with csv header delimiter ' ';"
  ```

## 地图制作
+ 处理采集的原始点文件
  1. 合并所有文件 merge_all.py
  2. 提取所有停止点 get_stop_points.py
  3. 以线为单位分割所有线点 splite_line.py

+ 构建拓扑地图(QGIS)
  1. 处理->工具箱->矢量创建->由点创建路径(point2path),id作为序号字段, l_id作为组字段, 点击运行,生成线.
  2. 对比停止点的位置，编辑线shp(目标将停止点作为节点，在停止点处打断，消除其他拓扑错误等等)

+ 向数据库中导入地图及属性信息
  1. 根据需要, 编辑属性. 使用编辑, 高级数字化工具栏, 捕捉工具栏.
  2. 转换格式 convert_to_db.py

### appendix
+ [PostgreSQL教程](https://www.runoob.com/postgresql/postgresql-tutorial.html)
+ [libpqxx tutorial](http://pqxx.org/development/libpqxx/)
+ [PostgreSQL - psql（客户端交互命令）](https://blog.csdn.net/guoxilen/article/details/41497575?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.nonecase)
+ [ubutnu postgresql 安装与配置](https://www.cnblogs.com/Siegel/p/6917213.html)