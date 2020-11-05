#ifndef RECEIVER_H
#define RECEIVER_H

#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <zcm/zcm-cpp.hpp>
#include "msg/include/structOBJECTLIST.hpp"
#include "msg/include/structESROBJINFO.hpp"
#include "msg/include/ESROBJECT.hpp"
#include "msg/include/structSICKMAP.hpp"
#include "msg/include/OBJECT.hpp"
#include "common/nature.h"

using namespace cv;
using namespace std;

typedef double transform_R[4][4];

class Handler
{
public:
  structOBJECTLIST objlist;
  structESROBJINFO esrobj;
  structSICKMAP sickmap;
  struct
      mutex objlistMutex,
      sickMutex, esrmutex;

  void handleObjlistMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structOBJECTLIST *msg)
  {
    objlistMutex.lock();
    objlist = *msg;
    objlistMutex.unlock();
  }

  void handleEsrObjMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structESROBJINFO *msg)
  {
    esrmutex.lock();
    esrobj = *msg;
    esrmutex.unlock();
  }

  void handleSickMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const structSICKMAP *msg)
  {
    sickMutex.lock();
    sickmap = *msg;
    sickMutex.unlock();
  }
};

void transform_identity(transform_R t)
{
  int r, c;

  for (r = 0; r < 4; r++)
    for (c = 0; c < 4; c++)
      if (r == c)
        t[r][c] = 1;
      else
        t[r][c] = 0;
}

void transform_left_multiply(transform_R t1, transform_R t2)
{
  transform_R result;
  int i, j, k;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
    {
      result[i][j] = 0;
      for (k = 0; k < 4; k++)
        result[i][j] += t2[i][k] * t1[k][j];
    }
  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      t1[i][j] = result[i][j];
}

void transform_rotate_z(transform_R t, double theta)
{
  transform_R temp;
  double ctheta = cos(theta), stheta = sin(theta);

  transform_identity(temp);
  temp[0][0] = ctheta;
  temp[0][1] = -stheta;
  temp[1][0] = stheta;
  temp[1][1] = ctheta;
  transform_left_multiply(t, temp);
}

void transform_point(double *x, double *y, double *z, transform_R t)
{
  double x2, y2, z2;

  x2 = t[0][0] * *x + t[0][1] * *y + t[0][2] * *z + t[0][3];
  y2 = t[1][0] * *x + t[1][1] * *y + t[1][2] * *z + t[1][3];
  z2 = t[2][0] * *x + t[2][1] * *y + t[2][2] * *z + t[2][3];
  *x = x2;
  *y = y2;
  *z = z2;
}

inline void MyLine(Mat img, Point start, Point end)
{
  int thickness = 1;
  int lineType = LINE_8;
  line(img,
       start,
       end,
       Scalar(255, 255, 0),
       thickness,
       lineType);
}

#endif