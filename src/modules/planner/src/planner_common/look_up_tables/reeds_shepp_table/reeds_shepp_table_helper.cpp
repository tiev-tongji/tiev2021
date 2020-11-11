#include "look_up_tables/reeds_shepp_table/reeds_shepp_table_helper.h"
#include "fstream"
#include "look_up_tables/binary_file_helper.h"
#include "look_up_tables/reeds_shepp_table/reeds_shepp.h"
#include "time.h"
#include <iostream>
#include <vector>
using namespace std;

const double PI = 3.1415926535898;
#define nameof(x) (#x)

namespace TiEV {

void ReedsSheppTableHelper::generate_distance_table_file(int r, double step, int angle_num, double turning_radius, string file_name) {
    long long stime = time(0);
    fstream   output(file_name, ios::out | ios::binary);
    if(!output.is_open() || angle_num <= 0 || r <= 0 || step <= 0 || turning_radius <= 0) {
        cerr << "File \"" << file_name << "\" not exist." << endl;
        return;
    }

    cout << nameof(r) << ": " << r << endl;
    cout << nameof(step) << ": " << step << endl;
    cout << nameof(angle_num) << ": " << angle_num << endl;
    cout << nameof(turning_radius) << ": " << turning_radius << endl;

    BinaryFileHelper::Write<int>(output, r);
    BinaryFileHelper::Write<double>(output, step);
    BinaryFileHelper::Write<int>(output, angle_num);
    BinaryFileHelper::Write<double>(output, turning_radius);

    vector<double> angles(angle_num);
    cout << "angles: ";
    for(int i = 0; i < angle_num; ++i) {
        double rad = (2.0 * PI) * i / angle_num;
        BinaryFileHelper::Write<double>(output, rad);
        cout << rad << " ";
        angles[i] = rad;  // real angles in x-y coordinate
    }

    ReedsSheppStateSpace rs(turning_radius);
    int                  data_count = r / step + 1;
    int                  prog       = data_count / 100;
    double               q1[3] = { 0 }, q2[3] = { 0 };
    cout << endl << endl;
    for(int count = 0; count < data_count; ++count) {
        if(count % prog == 0) {
            cout << "\rReading file : [" << count / prog << "%]";
            cout.flush();
        }
        for(int a1 = 0; a1 < angle_num; ++a1)
            for(int a2 = 0; a2 < angle_num; ++a2) {
                q1[2] = angles[a1];
                q2[2] = angles[a2];
                q2[0] = count * step;
                BinaryFileHelper::Write<double>(output, rs.distance(q1, q2));
            }
    }
    cout << endl << "Done in " << time(0) - stime << " secs" << endl;
    output.close();
}
}