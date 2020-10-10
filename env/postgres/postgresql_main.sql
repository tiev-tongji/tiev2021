--import shp into database
shp2pgsql -s 4326 -d /home/xlz/Desktop/map2/jiugongge_top_edited/jiugongge_topo_edited.shp jiugongge_road |psql -U postgres -d gis

--to change geomotry colunm srid 
update jiugongge_road set the_geom = st_geomfromtext(ST_AsText(the_geom),4326) 

--to check geomotry colunm srid 
select (ST_SRID(the_geom)) as srid from jiugongge_road   

--create topology
select pgr_createTopology('jiugongge_road',0.000001,'geom','gid');


ALTER TABLE jiugongge_road_single ADD COLUMN source integer;
ALTER TABLE jiugongge_road_single ADD COLUMN target integer;
ALTER TABLE jiugongge_road_single RENAME COLUMN geom TO the_geom;
ALTER TABLE jiugongge_road_single RENAME COLUMN shape_leng TO length;
ALTER TABLE jiugongge_road_single alter  COLUMN  length  type double precision ;


--check multilinestring number
SELECT COUNT(CASE WHEN ST_NumGeometries(the_geom) > 1 THEN 1 END) AS multi_geom,
COUNT(the_geom) AS total_geom
FROM jiugongge_road_single;

--change multilinestring to linestring
ALTER TABLE jiugongge_road_single
ALTER COLUMN the_geom TYPE geometry(linestring,4326) USING ST_GeometryN(the_geom, 1);

--shortest way query
drop table if exists shortpath;
select pgr_fromAtoB('jiugongge_road_single', 120.777555, 31.5892, 120.77749, 31.59298) into shortpath;


