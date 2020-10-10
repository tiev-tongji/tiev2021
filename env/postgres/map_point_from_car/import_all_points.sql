drop table if exists all_points;
CREATE TABLE all_points(
        p_id bigint,
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

COPY all_points FROM '/home/ljf/tiev2019/env/postgres/map_point_from_car/merge_all_edited_db.csv' delimiter ',';

update all_points 
set speed_mode = 4
where mode = 3;

drop table if exists stop_points;
CREATE TABLE stop_points(
        p_id bigint,
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

COPY stop_points FROM '/home/ljf/tiev2019/env/postgres/map_point_from_car/stop_points_db.csv' delimiter ',';
