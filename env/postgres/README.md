## 一次规划环境配置

### 依赖库安装

+ postgresql 安装
```
sudo apt-get install postgresql-11 libpq-dev postgresql-server-dev-11 postgresql-contrib
```
+ postgis 安装
```
sudo apt-get install postgis
```
+ pgrouting 安装
```
sudo apt-get install postgresql-11-pgrouting
```
+ libpqxx 安装
```
git clone https://github.com/jtv/libpqxx.git
mkdir build; cd build; camke ..; make; make install;(cmake版本最低为3.7)
```
+ pgadmin4 安装
```
sudo apt-get install pgadmin4
```

### 环境配置

+ postgresql 设置用户密码
```
sudo -u postgres -i
psql
\password
(please set password: postgres)
\q
```

+ postgresql 设置自动提交
```
sudo -u postgres -i
\set AUTOCOMMIT off
\q
```
+ postgresql 设置免密登陆
```
vim ./.pgpass
127.0.0.1:5432:gis:postgres:postgres
```

+ postgresql 插件加载
```
sudo -u postgres -i
createdb -E -UTF8 changshu
psql -c "CREATE EXTENSION hstore;" -d changshu
psql -c "CREATE EXTENSION fuzzystrmatch;" -d changshu
psql -c "CREATE EXTENSION postgis;" -d changshu
psql -c "CREATE EXTENSION postgis_topology;" -d changshu
psql -c "CREATE EXTENSION postgis_tiger_geocoder;" -d changshu
psql -c "CREATE EXTENSION pgrouting;" -d changshu
```
+ postgresql 函数加载
> pgadmin4连接数据库后执行sql(code_root/env/postgres/)，加载函数：
> shortestPath.sql
> pgr_pointoffset.sql
> pgr_updateTopoMap.sql
> outputPointPath.sql

### 导入地图文件和属性文件
+ import map file and create topo
```
shp2pgsql -s 4326 -d /home/autolab/tiev2019/env/postgres/map/changshu/jiugongge_topo_edit_2/jiugongge_topo_edited_2.shp jiugongge_road |psql -h 127.0.0.1 -p 5432 -U postgres -d changshu
```

```
ALTER TABLE jiugongge_road ADD COLUMN source integer,ADD COLUMN target integer;
ALTER TABLE jiugongge_road RENAME COLUMN geom TO the_geom;
SELECT pgr_createTopology('jiugongge_road', 0.000001, 'the_geom', 'gid');
ALTER TABLE jiugongge_road RENAME COLUMN shape_leng TO length;
ALTER TABLE jiugongge_road alter  COLUMN  length  type double precision;
ALTER TABLE jiugongge_road ALTER COLUMN the_geom TYPE geometry(linestring,4326) USING ST_GeometryN(the_geom, 1);
```

+ import property file

```
CREATE TABLE all_points(
        p_id bigint,
        heading real,
        curv real,
        mode bigint,
        speed_mode bigint,
        event_mode bigint,
        lane_num bigint,
        lane_seq bigint,
        lane_width bigint,
        geom geometry(Point, 4326)
      );

COPY all_points FROM 'your_map_points_file_path' delimiter ',';
```
## 地图制作
+ 处理采集的原始点文件
1. 合并所有文件 merge_all.py
2. 以线为单位分割所有线点 splite_line.py
3. 提取所有停止点 get_stop_points.py

+ 构建拓扑地图(Arcgis)
1. 点表->点shp->线shp(点集转线，以线id分段)->转为单部件线shp
2. 加载单部件线shp,对比停止点，编辑线shp(目标将停止点作为节点，在停止点处打断，消除其他拓扑错误等等)
3. 建立文件地理数据库->新建要素类->导入线shp->创建拓扑->拓扑检查->拓扑编辑->导出shp

+ 向数据库中导入地图及属性信息
1. 转换格式 convert_to_db.py
2. 入库 import_all_points.py