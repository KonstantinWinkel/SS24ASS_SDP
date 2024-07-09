#ifndef _NORMALS_H__
#define _NORMALS_H__

#include <vector>
#include "newmat/newmatap.h"
using namespace NEWMAT;

struct Point {
  double x;
  double y;
  double z;
};

double calcPlane(std::vector<Point> &ppoints, double plane[4]);
double calcPlane2(std::vector<Point> &ppoints, double plane[4], DiagonalMatrix& D,  Matrix& V);

#endif