#include <perception_types.h>
#include <grid.h>
#include <boost/thread/thread.hpp>
#include "msg/include/structFUSIONMAP.hpp"
#include <velocore.h>
#include "velodyne.h"
#ifdef MULTITHREAD
#include "omp.h"
#endif

#define    MAX_NUM_VELODYNE_SCANS   50000
#define    BINS_PER_REV             720
#define    MAX_BEAMS_IN_BIN         40  //10
#define    MAX_POINTS_PER_SCAN      8000
#define    CM_TO_METER_FACTOR       0.01
#define    VELODYNE_MIN_RANGE       2.0
#define    VELODYNE_MIN_RANGE_S     100//200
#define    NO_HEIGHT               -100.0
#define    MIN_DISTANCE_IN_CM       100
#define    VELO_BLIND_SPOT_START    17000
#define    VELO_BLIND_SPOT_STOP     19000
#define    MAX_COUNTER_DIFF         15

typedef struct {
    int                 num_beams;
    laser_point_p     * beam;
} beam_bin_t;

beam_bin_t           bin[NUM_LASER_BEAMS][BINS_PER_REV];
laser_scan_p         lscan;
laser_scan_p         nscan;
TiEV::VelodyneRings* rings;


typedef struct {
    int     idx;      /* velodyne index beam */
    int     pb;       /* partner beam for the comparison */
    float   v_angle;  /* vertical angle of the laser beam */
    float   h_angle;  /* horizontal angle of the laser beam */
    int     h_offset; /* horizontal offset of the beam in velodyne ticks */
    float   fac;      /* approximation factor of function with pb */
} velodyne_ring_settings_t;


#define OCCUPIED    1
#define FREE        0

#define NUM_SAMPLES    1000
#define EPSILON        0.000000001
#define MAX_RANGE      70.0

typedef __gnu_cxx::hash_multimap<uintptr_t, uintptr_t> map_type;

#ifdef USE_GRID_SEGMENTER
map_type cell_to_points;

#endif

void display_time(char* label, double time)
{
    double ms = ((dgc_get_time() - time)*1000);
    if (strlen(label) < 9)
        printf("#TIME: %s\t\t%02f", label, ms);
    else
        printf("#TIME: %s\t%02f", label, ms);

    if (ms > 20)
        printf(" * ");

    printf("\n");
}

void velodyne_init( dgc_velodyne_data_p v )
{
    v->num_scans = 0;
    v->scans = (dgc_velodyne_scan_p) malloc( MAX_NUM_VELODYNE_SCANS * sizeof(dgc_velodyne_scan_t));
    v->allocated_scans = MAX_NUM_VELODYNE_SCANS;
    dgc_velodyne_get_config(&v->config);
}

void set_cell_min( dgc_perception_map_cell_p cell,  float z, unsigned short counter )
{
    assert(cell != NULL);
//    if (counter_diff(cell->last_min,counter)>MAX_COUNTER_DIFF || cell->min > z) {
//        cell->min = z;
//        cell->last_min = counter;
//    }
    if ( cell->min > z) {
        cell->min = z;
        cell->last_min = counter;
    }
}

void set_cell_max( dgc_perception_map_cell_p cell,  float z, unsigned short counter )
{
    assert(cell != NULL);

//    if (counter_diff(cell->last_max,counter)>5) {
//        cell->max = z;
//        cell->last_max = counter;
//    } else if (z > cell->max && z-cell->min<settings.overpass_height) {
//        cell->max = z;
//        cell->last_max = counter;
//    }

//    if (counter_diff(cell->last_max,counter)>5) {
//        cell->max = z;
//        cell->last_max = counter;
//    } else
 if (z > cell->max){// && z-cell->min<settings.overpass_height) {
        cell->max = z;
        cell->last_max = counter;
    }
}


unsigned short max_valid_range[] =
        {
                0,    0,    0,    0,    0,    0,    0,    0,
                0,    0,    0,    0,    0,    0,    0,    0,
                0,    0,    0,    0,    0,    0,    0,    0,
                0,    0,    0,    0,    0,    0,    0,    0,
                1025, 1070, 0,    0,    1117, 1171, 868,  904,
                1245, 1336, 940,  996,  2225, 2400, 1431, 1473,
                2650, 3002, 1582, 1676, 3542, 4293, 1841, 2007,
                0,    0,    5453, 6522, 0,    0,    8513, 13529
        };
unsigned short max_valid_range_temp[] =
        {
                0,    0,    0,    0,    0,    0,    0,    0,
                0,    0,    0,    0,    0,    0,    0,    0,
                0,    0,    0,    0,    0,    0,    0,    0,
                0,    0,    0,    0,    0,    0,    0,    0,
                1025, 1070, 0,    0,    1117, 1171, 868,  904,
                1245, 1336, 940,  996,  2225, 2400, 1431, 1473,
                2650, 3002, 1582, 1676, 3542, 4293, 1841, 2007,
                0,    0,    5453, 6522, 0,    0,    8513, 13529
        };


void initialize(dgc_velodyne_data_p v)
{
    int                          i, l; //, size;

//#ifdef MULTITHREAD
//    omp_set_dynamic(0);
//    omp_set_num_threads(settings.num_threads);
//#endif

    for (l=0; l<NUM_LASER_BEAMS; l++) {
        for (i=0; i<BINS_PER_REV; i++) {//720
            bin[l][i].num_beams = 0;                     //10 now change to 40
            bin[l][i].beam      = (laser_point_p *) malloc( MAX_BEAMS_IN_BIN * sizeof(laser_point_p) );
        }
    }

    lscan = (laser_scan_p) malloc(NUM_LASER_BEAMS * sizeof(laser_scan_t));
    nscan = (laser_scan_p) malloc(NUM_LASER_BEAMS * sizeof(laser_scan_t));
    for (l=0; l<NUM_LASER_BEAMS; l++) {
        lscan[l].num_points = 0;
        lscan[l].laser_point = (laser_point_p) malloc( MAX_POINTS_PER_SCAN * sizeof(laser_point_t) );

        nscan[l].num_points = 0;
        nscan[l].laser_point = (laser_point_p) malloc( MAX_POINTS_PER_SCAN * sizeof(laser_point_t) );
    }

    rings = new TiEV::VelodyneRings(v->config, settings.velodyne_min_beam_diff);

    double theta = atan2(0.2, 1);
    double L = M_PI_2 + theta;
    double velodyne_height = v->config->offset[2][3] + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
    for (l=0; l<NUM_LASER_BEAMS; l++) {
        double R = v->config->vert_angle[l];
        if (R > -theta) {
            max_valid_range[l] = 0;
            continue;
        }
        R = M_PI_2 + R;

        double max_ground_range = 100.0 * velodyne_height * sin(L) / sin(M_PI - R - L);
        if (max_ground_range > std::numeric_limits<short>::max())
            max_valid_range[l] = 0;
        else
            max_valid_range[l] = (unsigned short) (max_ground_range);
    }
}


void process_velodyne(dgc_velodyne_data_p v)
{
    int l, i, j, e, b, n;
    if (!v->preprocessed) {
        laser_scan_p tmp = lscan;
        lscan = nscan;
        nscan = tmp;
        for (l = 0; l < NUM_LASER_BEAMS; l++) {
            lscan[l].num_points = 0;
        }
        for (l = 0; l < NUM_LASER_BEAMS; l++) {
            nscan[l].num_points = 0;
        }

        for( i = 0; i < v->num_scans; i++) {    
            int encoder = (v->scans[i].encoder + VELO_SPIN_START) % VELO_NUM_TICKS;//unwrap back to 0...36000
            encoder -= rings->minHorizontalOffset();
            for(j = 0; j < 32; j++) {
                unsigned short range = v->scans[i].p[j].range;

                l = j + v->scans[i].block * 32;
                e = encoder + rings->horizontalOffset(l);
                n = lscan[l].num_points;
                if (n<MAX_POINTS_PER_SCAN) {
                    lscan[l].laser_point[n].scan       = &(v->scans[i]);
                    lscan[l].laser_point[n].point      = &(v->scans[i].p[j]);
                    lscan[l].laser_point[n].z_cell     = NULL;
                    lscan[l].laser_point[n].encoder    = v->scans[i].encoder ;
                    if (sourcePointcloudEnable)
                    {
                        lscan[l].laser_point[n].obstacle   = TRUE;
                    }
                    else
                    {
                        lscan[l].laser_point[n].obstacle   = FALSE;
                    }
                    if (!v->config->laser_enabled[l])
                        lscan[l].laser_point[n].valid    = FALSE;
                    else
                        lscan[l].laser_point[n].valid    = TRUE;
                    lscan[l].num_points++;
                }
                lscan[l].robot = &(v->scans[i].robot);
            }
        }
        for (l=0; l<NUM_LASER_BEAMS; l++) {
            for (i=0; i<BINS_PER_REV; i++) {
                bin[l][i].num_beams = 0;
            }
        }
        int num=0;
        for(l=0; l<NUM_LASER_BEAMS; l++) {
            num+=lscan[l].num_points;
            for(i=0; i<lscan[l].num_points; i++) {
                if (lscan[l].laser_point[i].valid) {
                    e = lscan[l].laser_point[i].encoder;//0~36000   =>   0~720
                    b = (int)floor(e / ((float)VELO_NUM_TICKS/(float)BINS_PER_REV));
                    assert(bin[l][b].num_beams < MAX_BEAMS_IN_BIN);
                    bin[l][b].beam[bin[l][b].num_beams++] = &lscan[l].laser_point[i];
                }
            }
        }
        v->preprocessed = TRUE;
    } 
}

//TODO: cut from 8ms to 5ms
void label_obstacle_points_terrain(dgc_grid_p z_grid) {

//#define EXPAND_Z_GRID

#ifdef EXPAND_Z_GRID
    __gnu_cxx::hash_set<long> open;
#endif

    static bool init = true;
    static short obstacle_threshold = 0;

    obstacle_threshold = (short)(settings.z_obstacle_height / 0.01);

    static short max_z = std::numeric_limits<short>::max();
    int l;
    for (l = 0; l < NUM_LASER_BEAMS; l++) {
        for (int i = 0; i < lscan[l].num_points; i++) {
            short *grid_z = (short *) dgc_grid_get_xy(z_grid, lscan[l].laser_point[i].point->x,
                                                      lscan[l].laser_point[i].point->y);
            if (grid_z) {
                *grid_z = max_z;
                lscan[l].laser_point[i].z_cell = grid_z;
            }
        }
    }
    for (l = 0; l < NUM_LASER_BEAMS; l++) {
        for (int i = 0; i < lscan[l].num_points; i++) {
            short *grid_z = lscan[l].laser_point[i].z_cell;
            if (grid_z) {
                *grid_z = std::min(*grid_z, lscan[l].laser_point[i].point->z);
#ifdef EXPAND_Z_GRID
                open.insert((long) grid_z);//save z_grid cell which has been seen(from point_xy)
#endif
            }
        }
    }
#ifdef EXPAND_Z_GRID
    __gnu_cxx::hash_set<long>::iterator end = open.end();
    for (__gnu_cxx::hash_set<long>::iterator it = open.begin(); it != end; it++) {
        short *cell = (short *) *it;
        int r, c;
        dgc_grid_cell_to_rc_local(z_grid, cell, &r, &c);

        short *neighbor = (short *) dgc_grid_get_rc_local(z_grid, r + 1, c);
        if ((open.find((long) neighbor) != end) && (*cell > (*neighbor + 20)))
            *cell = *neighbor + 20;

        neighbor = (short *) dgc_grid_get_rc_local(z_grid, r - 1, c);
        if ((open.find((long) neighbor) != end) && (*cell > (*neighbor + 20)))
            *cell = *neighbor + 20;

        neighbor = (short *) dgc_grid_get_rc_local(z_grid, r, c - 1);
        if ((open.find((long) neighbor) != end) && (*cell > (*neighbor + 20)))
            *cell = *neighbor + 20;

        neighbor = (short *) dgc_grid_get_rc_local(z_grid, r, c + 1);
        if ((open.find((long) neighbor) != end) && (*cell > (*neighbor + 20)))
            *cell = *neighbor + 20;
    }
#endif
    for (int l = 0; l < NUM_LASER_BEAMS; l++) {
        for (int i = 0; i < lscan[l].num_points; i++) {
            short z = lscan[l].laser_point[i].point->z;
            short *grid_z = lscan[l].laser_point[i].z_cell;
            if ((grid_z) && ((z - *grid_z) > obstacle_threshold))
                if(lscan[l].laser_point[i].obstacle)
                    continue;
                else {
                    lscan[l].laser_point[i].obstacle = 1;
                }
        }
    }
}

void label_obstacle_cells(unsigned short counter, dgc_velodyne_data_p v)
{
    int l, b, k;
    float x, y, z, h;
    dgc_perception_map_cell_p cell;
#ifdef USE_GRID_SEGMENTER
   cell_to_points.clear();
#endif
    for (l=0; l<NUM_LASER_BEAMS-1; l++) {
        for(b = 0; b < BINS_PER_REV; b++) {
            for(k=0; k<bin[l][b].num_beams; k++) {
                if (bin[l][b].beam[k]&&bin[l][b].beam[k]->valid) {
                    x = bin[l][b].beam[k]->point->x * CM_TO_METER_FACTOR + bin[l][b].beam[k]->scan->robot.x;
                    y = bin[l][b].beam[k]->point->y * CM_TO_METER_FACTOR + bin[l][b].beam[k]->scan->robot.y;
                    //z = bin[l][b].beam[k]->point->z * CM_TO_METER_FACTOR + bin[l][b].beam[k]->scan->robot.z;

                    cell = (dgc_perception_map_cell_p)grid_get_xy(grid, x, y );
                    if(cell) {
#ifdef USE_GRID_SEGMENTER
                        {
                           cell_to_points.insert(map_type::value_type((uintptr_t)cell, (uintptr_t)bin[l][b].beam[k]));
                            ++cell->num_timestamps;
                            cell->timestamp_sum += bin[l][b].beam[k]->scan->timestamp;
                        }
#endif
                        z = bin[l][b].beam[k]->point->z * CM_TO_METER_FACTOR + bin[l][b].beam[k]->scan->robot.z;
                        if (z < -0.6 && z > -2.5) {
                            set_cell_min(cell, z, counter);//all the points in this cell to compute the cell->min and max
                            set_cell_max(cell, z, counter);
                            if(cell->max - cell->min > settings.z_obstacle_height || sourcePointcloudEnable){//height difference
                              cell->obstacle = 1;
                            }
                            cell->last_observed = counter;
                            if (cell->last_obstacle != counter) {
                                h = z - cell->min;
                                if (h < settings.overpass_height){
                                    if (bin[l][b].beam[k]->point->range > VELODYNE_MIN_RANGE_S) {
                                        if (bin[l][b].beam[k]->obstacle) {
                                            {
                                                cell->obstacle = 1;
                                            }
                                            cell->last_obstacle = counter;
                                            cell->hits += 2; 
                                        } else {
                                            cell->seen++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


// Add obstacles near car that the Velodyne can't see
void label_obstacle_cells_near(int counter, dgc_velodyne_data_p v,dgc_grid_p grid) {
    int num_cells_near_car = 0;
    static const int near_obstacle_buffer = 10;
    static const float near_obstacle_radius = 4.2;

    int origin_r, origin_c;
    dgc_transform_t t;
    double velodyne_x = 0.0, velodyne_y = 0.0, velodyne_z = 0.0;

    dgc_transform_rpy(t, v->config->offset, v->scans[0].robot.roll, v->scans[0].robot.pitch, v->scans[0].robot.yaw);
    dgc_transform_translate(t, v->scans[0].robot.x, v->scans[0].robot.y, v->scans[0].robot.z);
    dgc_transform_point(&velodyne_x, &velodyne_y, &velodyne_z, t);


    float velodyne_height = v->config->offset[2][3] + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT;
    // TODO: figure out why we need to add 40 cm here to get a realistic ground_z
    float ground_z = applanix_current_pose()->smooth_z - DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT + v->config->offset[2][3];

    dgc_perception_map_cell_p origin_cell = (dgc_perception_map_cell_p)grid_get_xy(grid, velodyne_x, velodyne_y );
    dgc_grid_cell_to_rc_local(grid, origin_cell, &origin_r, &origin_c);

    int invisible_radius = ceil(near_obstacle_radius / grid->resolution);
    int invisible_radius2 = invisible_radius * invisible_radius;
    for (int r = -invisible_radius; r <= invisible_radius; r++) {
        int r2 = r*r;
        for (int c = -invisible_radius; c <= invisible_radius; c++) {
            int c2 = c*c;
            int d2 = (r2 + c2);
            if (d2 > invisible_radius2)
                continue;
            dgc_perception_map_cell_p cell = (dgc_perception_map_cell_p)dgc_grid_get_rc_local(grid, origin_r + r, origin_c + c);
            if ( cell && (cell->last_observed != counter) && (cell->last_obstacle > 0) &&
                 ((cell->last_observed - cell->last_obstacle) < near_obstacle_buffer) &&
                 ((cell->last_obstacle - cell->last_dynamic) > near_obstacle_buffer) &&
                 (cell->hits > 2)) {
                // remove obstacles that we should be able to see because of their height,
                // this solves the problem of high obstacles passing through/near the occluded area and leaving behind static obstacles
                double distance = sqrt(d2) * grid->resolution;
                double max_z = ( (near_obstacle_radius-distance) / near_obstacle_radius) * velodyne_height + ground_z;
                cell->obstacle = 1;
                if (cell->max > max_z)
                    continue;
            }
        }
    }
//  printf("\nlabeled total %d cells as obstacles\n\n", obstacles_s->num);
}

//get point in the cell 
void points_in_cell(dgc_perception_map_cell_p cell, std::vector<point3d_t>& points) {
#ifdef USE_GRID_SEGMENTER
    points.reserve(cell_to_points.count((uintptr_t)cell));
    __gnu_cxx::pair<map_type::iterator, map_type::iterator> p = cell_to_points.equal_range((uintptr_t)cell);
    point3d_t point;
    for (map_type::iterator it = p.first; it != p.second; it++) {
        laser_point_p pt = (laser_point_p)(it->second);
        point.x = pt->point->x * CM_TO_METER_FACTOR + pt->scan->robot.x;
        point.y = pt->point->y * CM_TO_METER_FACTOR + pt->scan->robot.y;
        point.z = pt->point->z * CM_TO_METER_FACTOR + pt->scan->robot.z;
        point.intensity = pt->point->intensity;
        points.push_back(point);
    }
#endif
}

void set_object1(dgc_grid_p grid)
{
    dgc_perception_map_cell_p cell, cell_neighbor;

    //clear rain noisy in front car
    int top = 70;
    int bottom = 170;
    int left = 40;
    int right = 110;
    int searchSquare = 1;
    int clearMinCounter = 2;
    for (int i = top; i < bottom; i++) 
    {
        for (int j = left; j < right; j++) 
        {
            cell = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, i, j);
            if (cell->obstacle > 0) 
            {
                int conmn = 0;
                for (int m = -searchSquare; m < searchSquare + 1; m++)
                    for (int n = -searchSquare; n < searchSquare + 1; n++) {
                        cell_neighbor = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, i + m, j + n);
                        if (cell_neighbor->obstacle > 0) {
                            conmn++;
                        }
                    }
                if (conmn < clearMinCounter) {
                    cell->obstacle = 0;
                }
            }
        }
    }

    //clear noisy around car
    for (int i = -12; i < 12; i++) {
        if (i < -5) {
            for (int j = -5; j < 5; j++) {

                cell = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, TiEV::CAR_CEN_ROW + i, TiEV::CAR_CEN_COL + j);
                if (cell->obstacle > 0) {
                    cell->obstacle = 0;
                }
            }
        } else if (i < 8) {
            for (int j = -7; j < 8; j++) {
                cell = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, TiEV::CAR_CEN_ROW + i, TiEV::CAR_CEN_COL + j);
                if (cell->obstacle > 0) {
                    cell->obstacle = 0;
                }
            }
        } else if (i < 11) {
            for (int j = -5; j < 5; j++) {
                cell = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, TiEV::CAR_CEN_ROW + i, TiEV::CAR_CEN_COL + j);
                if (cell->obstacle > 0) {
                    cell->obstacle = 0;
                }
            }
        } else {
            for (int j = -2; j < 2; j++) {
                cell = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, TiEV::CAR_CEN_ROW + i, TiEV::CAR_CEN_COL + j);
                if (cell->obstacle > 0) {
                    cell->obstacle = 0;
                }
            }
        }
    }

    myzcm.myLaserMap.rows = grid->rows;
    myzcm.myLaserMap.cols  = grid->cols;
    myzcm.myLaserMap.resolution = grid->resolution;
    myzcm.myLaserMap.timestamp = TiEV::getTimeStamp();
    myzcm.myLaserMap.center_col = CAR_CEN_COL;
    myzcm.myLaserMap.center_row = CAR_CEN_ROW;
    myzcm.myLaserMap.utmX = latestNavInfo.utmX;
    myzcm.myLaserMap.utmY = latestNavInfo.utmY;
    myzcm.myLaserMap.mHeading = latestNavInfo.mHeading;

    //draw objstacle in cell
    for (int i = 0; i < grid->rows; i++)
    {
        for (int j = 0; j < grid->cols; j++) {
            cell = (dgc_perception_map_cell_p) dgc_grid_get_rc_local(grid, i, j);
            if (cell->obstacle > 0) {

                std::vector<point3d_t> cell_points;
                points_in_cell(cell, cell_points);
                for(int m = 0; m < cell_points.size(); ++m)
                {
                    //cloud for perception fusion location indoor
                    point3d pp;
                    pp.px_ = cell_points[m].x;
                    pp.py_ = cell_points[m].y;
                    pp.pz_ = cell_points[m].z;
                    sourceGridCloud.emplace_back(pp);
                }
                myzcm.myLaserMap.cells[TiEV::GRID_ROW - 1 - i][j] = cell->obstacle;//1 dynamic obstacle
                myVisual.image1.at<Vec3b>(TiEV::GRID_ROW - 1 - i, j)[0] = 255;
                myVisual.image1.at<Vec3b>(TiEV::GRID_ROW - 1 - i, j)[1] = 255;
                myVisual.image1.at<Vec3b>(TiEV::GRID_ROW - 1 - i, j)[2] = 255;
                obstacles_s->cell[obstacles_s->num++] = cell;
            }
        }
    }

    cout << "indoor point cloud size = " << sourceGridCloud.size() << endl;
    gridCloudMutex.lock();
    gridCloudFlag = true;
    gridCloudMutex.unlock();

    //draw car
    int carWidth = 5;
    int carHight = 15;
    for (int i = TiEV::CAR_CEN_ROW; i < TiEV::CAR_CEN_ROW + carHight + 1; ++i)
    {
        for (int j = TiEV::CAR_CEN_COL - carWidth; j < TiEV::CAR_CEN_COL + carWidth + 1; ++j)
        {
            myVisual.image1.at<Vec3b>(i,j)[0]=255;
            myVisual.image1.at<Vec3b>(i,j)[1]=255;
            myVisual.image1.at<Vec3b>(i,j)[2]=255;
        }
    }

    if (!is_mot_enable)
    {
        myVisual.show_image1();
        myzcm.pub_lasermap();
    }
}

//process only for lasermap
void integrate_velodyne( dgc_velodyne_data_p v, unsigned short counter)
{
    if (v->num_scans==0)
        return;
    static int                   firsttime = 1;
    if (firsttime) {
        initialize(v);//initialize
        firsttime = 0;
    }
    process_velodyne(v);//points to lscan and bin
    label_obstacle_points_terrain(z_grid);//decide which point belong to obstacle
    label_obstacle_cells(counter, v);//decide which cell belong to obstacle
    //label_obstacle_cells_near(counter, v ,grid);
    set_object1(grid); //publish
}


dgc_pose_t computeAverageRobotPoseForCells(const std::vector<dgc_perception_map_cell_p>& cells)
{
#ifndef USE_GRID_SEGMENTER
    bool this_function_requires_grid_segmenter = false;
  assert(this_function_requires_grid_segmenter);
#endif

    dgc_pose_t robot;
    robot.x = 0;
    robot.y = 0;
    robot.z = 0;
    robot.roll = 0;
    robot.pitch = 0;
    robot.yaw = 0;

    double count = 0;
    for(size_t i = 0; i < cells.size(); ++i) {
        __gnu_cxx::pair<map_type::iterator, map_type::iterator> p = cell_to_points.equal_range((uintptr_t)cells[i]);
        for (map_type::iterator it = p.first; it != p.second; it++) {
            laser_point_p pt = (laser_point_p)(it->second);
            robot.x += pt->scan->robot.x;
            robot.y += pt->scan->robot.y;
            robot.z += pt->scan->robot.z;
            robot.roll += pt->scan->robot.roll;
            robot.pitch += pt->scan->robot.pitch;
            robot.yaw += pt->scan->robot.yaw;

            ++count;
        }
    }

    assert(count != 0);
    robot.x /= count;
    robot.y /= count;
    robot.z /= count;
    robot.roll /= count;
    robot.pitch /= count;
    robot.yaw /= count;

    return robot;
}
