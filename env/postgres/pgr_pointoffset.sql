--DROP FUNCTION pgr_pointoffset(character varying,double precision,double precision);
CREATE OR REPLACE function pgr_pointoffset(tbl varchar,blockedx float, blockedy float)   
--returns geometry as  
returns setof float as 
$body$  
declare  
    v_blockedLine geometry;--被阻断的线 
    point_in_line geometry;
    endpoint geometry;
    targetpoint geometry;
    id integer;--被阻断的线  
    start_x float;
    start_y float;
    sin float;
    cos float;
    end_x float;
    end_y float;
    target_x float;
    target_y float;
begin     
    --查询离起点最近的线  
    execute 'select the_geom ,gid  from '|| tbl ||' 
                where 
            ST_DWithin(the_geom,ST_Geometryfromtext(''point('|| blockedx ||' ' || blockedy||')'',4326),15) 
            order by ST_Distance(the_geom,ST_GeometryFromText(''point('|| blockedx ||' '|| blockedy ||')'',4326))  limit 1' 
            into v_blockedLine ,id;  

    select ST_LineInterpolatePoint(v_blockedLine, ST_LineLocatePoint(v_blockedLine ,ST_GeometryFromText('point('|| blockedx ||' ' || blockedy ||')',4326))) into point_in_line;
    --select ST_LineInterpolatePoint( v_blockedLine ||, ST_LineLocatePoint( v_blockedLine ||',''point('|| blockedx ||' ' || blockedy ||')''))' into point_in_line;
    --execute 'select ST_LineInterpolatePoint('|| v_blockedLine ||', ST_LineLocatePoint('|| v_blockedLine ||',ST_GeometryFromText(''point('|| blockedx ||' ' || blockedy ||')'',4326)))' into point_in_line;
    select st_x(ST_Transform(point_in_line,900913)) into start_x;
    select st_y(ST_Transform(point_in_line,900913)) into start_y;

    select ST_EndPoint(v_blockedLine) into  endpoint;
    select st_x(ST_Transform(endpoint,900913)) into end_x;
    select st_y(ST_Transform(endpoint,900913)) into end_y;

    sin = (end_x - start_x) / (|/ ((start_y - end_y)^2+(start_x - end_x)^2));
    cos = -(end_y - start_y) / (|/ ((start_y - end_y)^2+(start_x - end_x)^2));


    target_x = start_x + 5*cos;
    target_y = start_y + 5*sin;

    select ST_Transform(ST_Geometryfromtext('point('|| target_x ||' ' || target_y||')',900913),4326) into targetpoint;
    select st_x(targetpoint) into target_x;
    select st_y(targetpoint) into target_y;


    return next target_x;
    return next target_y;
    --return targetpoint;

end;  
$body$  
LANGUAGE plpgsql VOLATILE STRICT
--select * from pgr_pointoffset('jiugongge_road', 120.77638 , 31.58953) ;
