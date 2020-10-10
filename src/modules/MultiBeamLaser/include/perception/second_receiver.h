#pragma once 

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <climits>
#include <vector>
#include <time.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "perception.h"
#include "obstacle.h"

#include <Python.h>
#include <abstract.h>
// NumPy C/API headers
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION // remove warnings
#include <numpy/ndarrayobject.h>


using namespace Eigen;
using namespace std;

class SecondPython
{
    public:
        SecondPython();
        void startReceiver(vector<float> &buffer, double timestamp);
        ~SecondPython();
    private:
        PyObject *pymodule;
        PyObject *pyfunction;
};

