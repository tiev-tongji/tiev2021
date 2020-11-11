#include "look_up_tables/distance_table.h"
#include "look_up_tables/binary_file_helper.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <unistd.h>

using namespace std;

namespace TiEV {

const double PI      = 3.1415926535898;
const double INF     = 1e10;
const double EPSILON = 1e-6;
#define nameof(x) (#x)

DistanceTable::DistanceTable(string table_file_name) {
    fstream input(table_file_name, ios::in | ios::binary);
    if(!input.is_open()) {
        cerr << "File \"" << table_file_name << "\" not exist." << endl;
        return;
    }

    BinaryFileHelper::Read<int>(input, r);
    BinaryFileHelper::Read<double>(input, step);
    BinaryFileHelper::Read<int>(input, angle_num);
    BinaryFileHelper::Read<double>(input, turning_radius);

    if(!input || angle_num <= 0 || r <= 0 || step <= 0 || turning_radius <= 0) {
        cerr << "Illegal file format." << endl;
        return;
    }

    cout << "--------------------" << endl << "Distance Table" << endl << "--------------------" << endl;

    cout << nameof(r) << ": " << r << endl;
    cout << nameof(step) << ": " << step << endl;
    cout << nameof(angle_num) << ": " << angle_num << endl;
    cout << nameof(turning_radius) << ": " << turning_radius << endl;

    cout << nameof(angle_num) << ": " << angle_num << endl;
    angles = new double[angle_num];
    for(int i = 0; i < angle_num; ++i)
        BinaryFileHelper::Read<double>(input, angles[i]);

    int data_count = r / step + 1;

    mem = new double[1ll * data_count * angle_num * angle_num];
    cout << endl;
    cout.flush();
    int prog = data_count / 100;
    for(int count = 0; count < data_count; ++count) {
        if(count % prog == 0) {
            cout << "\rReading file : [" << count / prog << "%]";
            cout.flush();
        }
        for(int a1 = 0; a1 < angle_num; ++a1)
            for(int a2 = 0; a2 < angle_num; ++a2)
                BinaryFileHelper::Read<double>(input, mem[getPos(count, a1, a2)]);
    }
    cout << "\rReading file : [100%]" << endl;
    if(input)
        input.close();
    else
        cerr << "Incomplete file." << endl;
}

long long DistanceTable::getPos(int count, int a1, int a2) const {
    return (count * angle_num + a1) * angle_num + a2;
}

double DistanceTable::getDistance(double q0[3], double q1[3]) const {
    double dx = q1[0] - q0[0], dy = q1[1] - q0[1];
    double length = sqrt(dx * dx + dy * dy);
    int    count  = round(length / step);
    if(count > (r / step)) return INF;
    // Rotate vector (q0, q1) to a vertical vector
    if(length < EPSILON) return 0;
    double       theta = atan2(dy, dx);
    const double inter = (2 * PI / angle_num);
    double       a1 = q0[2] - theta, a2 = q1[2] - theta;
    while(a1 < 0)
        a1 += 2 * PI;
    while(a2 < 0)
        a2 += 2 * PI;
    while(a1 >= 2 * PI)
        a1 -= 2 * PI;
    while(a2 >= 2 * PI)
        a2 -= 2 * PI;
    int a1i = round(a1 / inter), a2i = round(a2 / inter);
    return mem[getPos(count, a1i % angle_num, a2i % angle_num)];
}

DistanceTable::~DistanceTable() {
    delete mem;
}
}