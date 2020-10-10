#include <iostream>
#include <pqxx/pqxx>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <boost/program_options.hpp>
namespace  bpo = boost::program_options;

int main(int argc, char const *argv[])
{

  /*options parser*/
  double startPointX = 120.777555;
  double startPointY = 31.5892;
  double endPointX = 120.77749;
  double endPointY = 31.59298;
  double blockedX = 0.0;
  double blockedY = 0.0;
  int mode = 0;
  bool is_initmap = true;
  std::string shpfile, output, order, database, password;
  std::string user = "postgres";

  clock_t  clockBegin, clockEnd;  
 

  bpo::options_description opts("all options"); 
  bpo::variables_map vm;
  opts.add_options()
  ("shpfile,s", bpo::value<std::string>(&shpfile), "the path of shapefile")
  ("user,u", bpo::value<std::string>(&user), "username default is postgres")
  ("database,d", bpo::value<std::string>(&database), "username default is postgres")
  ("password,p", bpo::value<std::string>(&password), "password of you database")
  ("outputTxt,o", bpo::value<std::string>(&output), "the path of output file")
  ("spx", bpo::value<double>(&startPointX), "the COOR_X of start point")
  ("spy", bpo::value<double>(&startPointY), "the COOR_Y of start point")
  ("epx", bpo::value<double>(&endPointX), "the COOR_X of end point")
  ("epy", bpo::value<double>(&endPointY), "the COOR_Y of end point")
  ("bpx", bpo::value<double>(&blockedX), "the COOR_X of blocked point")
  ("bpy", bpo::value<double>(&blockedY), "the COOR_Y of blocked point")
  ("mode,m", bpo::value<int>(&mode), "work mode, update: 1; planning path:2 ")
  //("isupdate,upd", bpo::value<bool>(&is_update), "update topo map, true or false")
  ("isinitmap,ini", bpo::value<bool>(&is_initmap), "initial topo map, true or false")
  ("help,h", "this is a program to find a shortest GPS point path");
  try{
      bpo::store(bpo::parse_command_line(argc, argv, opts), vm);
  }
  catch(...){
      std::cout << "option do not exist\n";
      return 0;
  }

  bpo::notify(vm);  

  if(vm.empty() ){
      std::cout << "no options found \n";
      return 0;
  }
  if(vm.count("help") ){
    std::cout << opts << std::endl;
    return 0;
  }
  /************************************************/


  try
  { 

    /*connection to database */
    /*******************************************************/
    order = "dbname=" + database + " user=" + user + " password=" + password + " hostaddr=127.0.0.1 port=5432";
    pqxx::connection c(order);
    if (c.is_open()) {
      std::cout << "Connection succesful!" << std::endl;   
    }
    else {
      std::cout << "Something went wrong... oops" << std::endl;  
    }
    pqxx::work w(c);
    pqxx::result r;
    
    /*add column source and target and create topology*/
    /**************************************************/
    if (is_initmap)
    {
      /*import shapfile to postgres*/
      /*******************************************************/
      clockBegin = clock();  
      //w.exec("drop table if exists jiugongge_road;");
      //w.exec("drop table if exists jiugongge_road_vertices_pgr;");
      order = "shp2pgsql -s 4326 -d " + shpfile + " jiugongge_road |psql -h 127.0.0.1 -p 5432 -U " + user + " -d " + database;
      system(order.c_str());
      std::cout << "import shapefile successfully!" << std::endl;

      w.exec("ALTER TABLE jiugongge_road ADD COLUMN source integer,ADD COLUMN target integer;");
      w.exec("ALTER TABLE jiugongge_road RENAME COLUMN geom TO the_geom;");
      w.exec( "SELECT pgr_createTopology('jiugongge_road', 0.000001, 'the_geom', 'gid');");
      w.exec("ALTER TABLE jiugongge_road RENAME COLUMN shape_leng TO length;");
      w.exec("ALTER TABLE jiugongge_road alter  COLUMN  length  type double precision;");
      w.exec("ALTER TABLE jiugongge_road ALTER COLUMN the_geom TYPE geometry(linestring,4326) USING ST_GeometryN(the_geom, 1);");
      std::cout << "topology created successfully!" << std::endl;
      clockEnd = clock();  
      printf("%d\n", int(clockEnd - clockBegin));
    }
    
    const char * sql = nullptr;
    /*sql = "ALTER TABLE jiugongge_road "\
    "ADD COLUMN source integer,"\
    "ADD COLUMN target integer;"\
    "ALTER TABLE jiugongge_road RENAME COLUMN geom TO the_geom;"\
    "SELECT pgr_createTopology('jiugongge_road', 0.000001, 'the_geom', 'gid');"\
    "ALTER TABLE jiugongge_road RENAME COLUMN shape_leng TO length;"\
    "ALTER TABLE jiugongge_road alter  COLUMN  length  type double precision;"\
    "ALTER TABLE jiugongge_road "\
    "ALTER COLUMN the_geom TYPE geometry(linestring,4326) USING ST_GeometryN(the_geom, 1);";
    w.exec(sql);
    */

    if (mode == 2)
    {
      /*calculate the shortest path between A and B*/
      /**************************************************/
      clockBegin = clock(); 
      const char * sql_shortPath = nullptr;
      char sqlOrder[] = "select pgr_fromAtoB('jiugongge_road', %f, %f, %f, %f) into shortpath;";
      char buf[strlen(sqlOrder)];
      sprintf(buf, sqlOrder, startPointX, startPointY, endPointX, endPointY); 
      sql_shortPath = buf;
      w.exec("drop table if exists shortpath;");
      w.exec(sql_shortPath);
      std::cout << "shortpath created successfully!" << std::endl;

      /*generate GPS path point in shortpath*/
      /**************************************************/
      const char* sql_txt = nullptr;
      order = "select line2points('shortpath',0.5,'jiugongge_road_vertices_pgr');";
      sql_txt = order.c_str();
      w.exec(sql_txt);
      clockEnd = clock();
      printf("%d\n", int(clockEnd - clockBegin));


      //sql_txt = nullptr;
      clockBegin = clock();
      order = "drop table if exists point_in_buffer;"\
              "SELECT * into point_in_buffer FROM all_points "\
              "WHERE ST_Within(geom, (SELECT ST_Buffer((SELECT pgr_fromatob FROM shortpath),0.00001,'endcap=round join=round')));";
      sql_txt = order.c_str();
      w.exec(sql_txt);
      std::cout<< "build buffer successfully!"<<std::endl;
      clockEnd = clock();
      printf("%d\n", int(clockEnd - clockBegin));

      clockBegin = clock();
      order = "drop table if exists result_path_points;"\
              "SELECT * into result_path_points FROM("\
              "SELECT p.id, p.lon, p.lat, b.z, b.curv, b.velo_mode, b.event_mode "\
                    "FROM result_points_tbl p "\
                    "CROSS JOIN LATERAL ( "\
                        "SELECT r.z, r.curv, r.velo_mode, r.event_mode "\
                        "FROM point_in_buffer r "\
                        "ORDER BY r.geom <-> p.point "\
                        "LIMIT 1 "\
                    ")b "\
                ")AS table1;"\
                "drop table if exists result_points_tbl;"\
                "COPY (SELECT * FROM result_path_points) TO '"+ output + "' (format csv, delimiter ' ');";
      sql_txt = order.c_str();
      w.exec(sql_txt);



      std::cout<< "output successfully! planned path saved in /opt."<<std::endl;
      clockEnd = clock();
      printf("%d\n", int(clockEnd - clockBegin));
      //std::cout<< "change output file's permission, please input password"<< std::endl;
      //order = "sudo chmod 777 " + output;
      //system(order.c_str()); 
      //std::cout<< "drop!"<<std::endl;
      //sql = "DROP TABLE jiugongge_road_vertices_pgr;";
            //"DROP TABLE jiugongge_road;"
      //w.exec(sql);

    }

    if (mode == 1 and blockedX and blockedY)
    {
      /*update topo_map when the road was blocked*/
      /**************************************************/
      clockBegin = clock();
      const char * sql_updateMap = nullptr;
      char updateOrder[] = "select pgr_updateTopoMap('jiugongge_road', %f, %f);";
      char updatebuf[strlen(updateOrder)];
      sprintf(updatebuf, updateOrder, startPointX, startPointY); 
      sql_updateMap = updatebuf;
      w.exec(sql_updateMap);
      std::cout << "update map successfully!" << std::endl;
      clockEnd = clock();
      printf("%d\n", int(clockEnd - clockBegin));
    }

    w.commit();
    std::cout<<"commit!"<<std::endl;
    c.disconnect(); 
    std::cout<<"disconnect!"<<std::endl;
    
  }
  catch (const pqxx::sql_error &e)
  {
    std::cerr
        << "Database error: " << e.what() << std::endl
        << "Query was: " << e.query() << std::endl;
    return 2;
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}