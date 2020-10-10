CREATE OR REPLACE function pgr_updateTopoMap(tbl varchar,blockedx float, blockedy float)   
returns  void as  
$body$  
declare  
    v_blockedLine geometry;--被阻断的线 
       
    id integer;--被阻断的线  
begin     
          
    --查询离起点最近的线  
    execute 'select the_geom ,gid  from '||tbl||' 
                where 
            ST_DWithin(the_geom,ST_Geometryfromtext(''point('|| blockedx ||' ' || blockedy||')'',4326),15) 
            order by ST_Distance(the_geom,ST_GeometryFromText(''point('|| blockedx ||' '|| blockedy ||')'',4326))  limit 1' 
            into v_blockedLine ,id;  

    execute 'UPDATE '||tbl||'
            SET length = 99999
            WHERE gid = '||id||' ';
  
      
end;  
$body$  
LANGUAGE plpgsql VOLATILE STRICT
