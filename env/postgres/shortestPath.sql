CREATE OR REPLACE function pgr_fromAtoB(tbl varchar,startx float, starty float,endx float,endy float)   
returns  geometry as  
$body$  
declare  
    v_startLine geometry;--离起点最近的线  
    v_endLine geometry;--离终点最近的线  
      
    v_startTarget integer;--距离起点最近线的终点  
    v_endSource integer;--距离终点最近线的起点  
  
    v_statpoint geometry;--在v_startLine上距离起点最近的点  
    v_endpoint geometry;--在v_endLine上距离终点最近的点  
      
    v_res geometry;--最短路径分析结果  
  
  
    v_perStart float;--v_statpoint在v_res上的百分比  
    v_perEnd float;--v_endpoint在v_res上的百分比  
  
    v_shPath geometry;--最终结果
    tempnode float; 
begin     
          
    --查询离起点最近的线  
    execute 'select the_geom ,target  from ' ||tbl||
            ' where 
            ST_DWithin(the_geom,ST_Geometryfromtext(''point('|| startx ||' ' || starty||')'',4326),15) 
            order by ST_Distance(the_geom,ST_GeometryFromText(''point('|| startx ||' '|| starty ||')'',4326))  limit 1' 
            into v_startLine ,v_startTarget;  
      
    --查询离终点最近的线  
    execute 'select the_geom,source  from ' ||tbl||
            ' where ST_DWithin(the_geom,ST_Geometryfromtext(''point('|| endx || ' ' || endy ||')'',4326),15) 
            order by ST_Distance(the_geom,ST_GeometryFromText(''point('|| endx ||' ' || endy ||')'',4326))  limit 1' 
            into v_endLine,v_endSource;  
  
    --如果没找到最近的线，就返回null  
    if (v_startLine is null) or (v_endLine is null) then  
        return null;  
    end if ;  
  
    select  ST_ClosestPoint(v_startLine, ST_Geometryfromtext('point('|| startx ||' ' || starty ||')',4326)) into v_statpoint;  
    select  ST_ClosestPoint(v_endLine, ST_GeometryFromText('point('|| endx ||' ' || endy ||')',4326)) into v_endpoint;  
  
      
    --最短路径  
    execute 'SELECT st_linemerge(st_union(b.the_geom))' ||
    'FROM pgr_dijkstra(  
    ''SELECT gid as id, source, target, length as cost FROM ' || tbl ||''' ,' ||
    v_startTarget || ',' ||v_endSource || ', true  ) a,' || tbl || ' b    
    WHERE a.edge = b.gid'
    into v_res;  
    
  
    --如果找不到最短路径，就返回null  
    --if(v_res is null) then  
    --    return null;  
    --end if;  
      
    --将v_res,v_startLine,v_endLine进行拼接  
    select  st_linemerge(ST_Union(array[v_res,v_startLine,v_endLine])) into v_res;  
      
    select  ST_LineLocatePoint(v_res, v_statpoint) into v_perStart;  
    select  ST_LineLocatePoint(v_res, v_endpoint) into v_perEnd;  
    
    if(v_perStart > v_perEnd) then  
        tempnode =  v_perStart;
        v_perStart = v_perEnd;
        v_perEnd = tempnode;
    end if;
    
    --截取v_res  
    SELECT ST_LineSubString(v_res,v_perStart, v_perEnd) into v_shPath;  
        
    return v_shPath;  
  
      
      
end;  
$body$  
LANGUAGE plpgsql VOLATILE STRICT
