CREATE OR REPLACE function pgr_fromAtoB_unnormal(tbl varchar, vertices varchar, blockedx float, blockedy float, startx float, starty float,endx float,endy float)   
returns  geometry as  
$body$  
declare  
    v_startLine geometry;--离起点最近的线  
    v_endLine geometry;--离终点最近的线  
    v_blockedLine geometry; --堵塞的线
      
    v_startTarget integer;--距离起点最近线的终点  
    v_startSource integer;--距离起点最近线的起点
    v_endSource integer;--距离终点最近线的起点  
    v_bloackedTarget integer;--距离堵塞点最近线的终点
    
    v_startSourceGeom geometry;
    v_blockedTargetGeom geometry;
    v_linebetweenBTSS geometry;
  
    v_statpoint geometry;--在v_startLine上距离起点最近的点  
    v_endpoint geometry;--在v_endLine上距离终点最近的点  
    v_blockedpoint geometry; --在v_blockedLine上距离堵塞点最近的点
      
    v_res geometry;--最短路径分析结果 
    v_shPathAdd geometry;--合并额外道路
    v_startLineRes geometry;--startLine上的额外道路
    v_blockedLineRes geometry;--blockedLine上的额外道路
  
    v_perStart float;--v_statpoint在v_res上的百分比  
    v_perEnd float;--v_endpoint在v_res上的百分比  
    v_perBlockedPoint float;
    v_perStartLineStart float;

  
    v_shPath geometry;--最终结果
    tempnode float; 
begin     
          
    --查询离起点最近的线   
    execute 'select the_geom ,target, source  from ' ||tbl||
            ' where 
            ST_DWithin(the_geom,ST_Geometryfromtext(''point('|| startx ||' ' || starty||')'',4326),15) 
            order by ST_Distance(the_geom,ST_GeometryFromText(''point('|| startx ||' '|| starty ||')'',4326))  limit 1' 
            into v_startLine ,v_startTarget, v_startSource; 

    execute 'select the_geom from ' || vertices ||
            ' where id = ' || v_startSource
            into v_startSourceGeom;
      
    --查询离终点最近的线  
    execute 'select the_geom,source  from ' ||tbl||
            ' where ST_DWithin(the_geom,ST_Geometryfromtext(''point('|| endx || ' ' || endy ||')'',4326),15) 
            order by ST_Distance(the_geom,ST_GeometryFromText(''point('|| endx ||' ' || endy ||')'',4326))  limit 1' 
            into v_endLine,v_endSource; 

    --查询离堵塞点最近的线 
    execute 'select the_geom,target  from ' ||tbl||
            ' where ST_DWithin(the_geom,ST_Geometryfromtext(''point('|| blockedx || ' ' || blockedy ||')'',4326),15) 
            order by ST_Distance(the_geom,ST_GeometryFromText(''point('|| blockedx ||' ' || blockedy ||')'',4326))  limit 1' 
            into v_blockedLine,v_bloackedTarget; 

    execute 'select the_geom from ' || vertices ||
            ' where id = ' || v_bloackedTarget
            into v_blockedTargetGeom;
  
    --如果没找到最近的线，就返回null  
    if (v_startLine is null) or (v_endLine is null) then  
        return null;  
    end if;  
  
    select  ST_ClosestPoint(v_startLine, ST_Geometryfromtext('point('|| startx ||' ' || starty ||')',4326)) into v_statpoint;  
    select  ST_ClosestPoint(v_endLine, ST_GeometryFromText('point('|| endx ||' ' || endy ||')',4326)) into v_endpoint;  
    select  ST_ClosestPoint(v_blockedLine, ST_GeometryFromText('point('|| blockedx ||' ' || blockedy ||')',4326)) into v_blockedpoint;
  
      
    --最短路径  
    execute 'SELECT st_linemerge(st_union(b.the_geom))' ||
    'FROM pgr_dijkstra(  
    ''SELECT gid as id, source, target, length as cost FROM ' || tbl ||''' ,' ||
    v_startTarget || ',' ||v_endSource || ', true  ) a,' || tbl || ' b    
    WHERE a.edge = b.gid'
    into v_res;  
  
    --如果找不到最短路径，就返回null  
    if(v_res is null) then  
        return null;  
    end if;  
      
    --将v_res,v_startLine,v_endLine进行拼接  
    select  st_linemerge(ST_Union(array[v_res,v_startLine,v_endLine])) into v_res;  
      
    select  ST_LineLocatePoint(v_res, v_statpoint) into v_perStart;  
    select  ST_LineLocatePoint(v_res, v_endpoint) into v_perEnd;  
    select  ST_LineLocatePoint(v_blockedLine, v_blockedpoint) into v_perBlockedPoint;  
    select  ST_LineLocatePoint(v_startLine, v_statpoint) into v_perStartLineStart;
    select  ST_MakeLine(v_blockedTargetGeom, v_startSourceGeom) into v_linebetweenBTSS;
    
    if(v_perStart > v_perEnd) then  
        tempnode =  v_perStart;
        v_perStart = v_perEnd;
        v_perEnd = tempnode;
    end if;

    --截取v_res  
    SELECT ST_LineSubString(v_res,v_perStart, v_perEnd) into v_shPath;
    SELECT ST_LineSubString(v_blockedLine,v_perBlockedPoint,1.0) into v_blockedLineRes;
    SELECT ST_LineSubString(v_startLine,0.0,v_perStartLineStart) into v_startLineRes;

    SELECT st_linemerge(ST_Union(array[v_blockedLineRes,v_linebetweenBTSS,v_startLineRes,v_shPath])) into v_shPathAdd;
        
    return v_shPathAdd;  

end;  
$body$  
LANGUAGE plpgsql VOLATILE STRICT
