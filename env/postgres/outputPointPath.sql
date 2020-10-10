--select line2points('shortpath',0.5,'ways_vertices_pgr');
--COPY (select * from result_points_tbl) TO '/tmp/sql_output.txt' (format csv, delimiter '	')
--CREATE TABLE ways3 AS (SELECT * FROM ways2)


CREATE OR REPLACE function line2points(tbl varchar,distance float,link_points_tbl varchar) returns  void as 
$body$ 
declare 
	ii integer; -- 循环次数
	n integer;
	point_temp geometry;--内插点 
	longtitude double precision;
	latitude double precision;
	id_temp integer;
	 
	begin
	drop table if exists result_points_tbl;
	CREATE TABLE result_points_tbl(id integer,point geometry,lon double precision ,lat double precision);

	execute 'select ST_NPoints(pgr_fromatob) from ' || tbl|| ' ' into n;

	ii:=0;
	FOR ii IN 0..n LOOP

	execute 'SELECT ST_PointN(pgr_fromatob, ' ||ii|| ') FROM ' ||tbl|| ' ' into point_temp;
		   
	SELECT st_x(point_temp) into longtitude;
	SELECT st_y(point_temp) into latitude;
	insert into result_points_tbl(id,point,lon,lat) values(ii,point_temp,longtitude,latitude);
	end loop;
end;
$body$ 
LANGUAGE plpgsql VOLATILE STRICT
