/*
 * Copyright (c) 2018 TiEV (Tongji Intelligent Electric Vehicle).
 *
 * This file is part of TiEV Autonomous Driving Software
 * (see cs1.tongji.edu.cn/tiev).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include "smooth.h"
#include <cstdlib>
#include <cmath>

using namespace std;

namespace TiEV {

double SmoothStep(double orivalue,
    double &newvalue,
    double prenewvalue,
    double nextnewvalue,
    double weight_data,
    double weight_smooth){

    double aux = newvalue;
    newvalue += weight_data * (orivalue - newvalue);
    newvalue += weight_smooth * (prenewvalue + nextnewvalue - (2.0 * newvalue));
    return fabs(aux - newvalue);
}

//Added for smoothing path using CS373 smoother algorithm by Junqiao Zhao
std::vector<std::pair<double, double>> Smoother::quadricSmoother(
    const vector<pair<double, double>> &pointlist,
    const double &weight_data,
    const double &weight_smooth,
    const double &tolerance){

    vector<pair<double, double>> newpointlist(pointlist);

    double change = tolerance;
    while (change >= tolerance){
      	change = 0.0;
      	for (int i = 1; i < (int)newpointlist.size() - 1; ++i){
      	  	//first
      	  	change += SmoothStep(
      	      	pointlist[i].first,
      	      	newpointlist[i].first,
      	      	newpointlist[i - 1].first,
      	      	newpointlist[i + 1].first,
      	      	weight_data,
      	      	weight_smooth);

      	  	//second
      	  	change += SmoothStep(
      	      	pointlist[i].second,
      	      	newpointlist[i].second,
      	      	newpointlist[i - 1].second,
      	      	newpointlist[i + 1].second,
      	      	weight_data,
      	      	weight_smooth);
      	}
    }

    return newpointlist;
}

}
