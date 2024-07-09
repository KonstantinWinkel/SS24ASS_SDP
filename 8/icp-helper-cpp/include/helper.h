#ifndef _HELPER_H_
#define _HELPER_H__

#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "point.h"
#include "newmat/newmatap.h"
using namespace NEWMAT;

struct PtPair {
  Point p1;
  Point p2;
};

double rad(const double deg);

double dist2(Point p1, Point p2); 

double sqr(double d);

void print(std::string filename, std::vector<Point> &points);

void transform(Point &p, const Matrix t);

double alignSVD(const vector<PtPair>& pairs, Matrix &alignfx, bool quiet=true);

Matrix computeTransformation2(double theta, double dx, double dy);

void transformCloud(vector<Point> &points, Matrix &mat);

void copyCloud(vector<Point> &in, vector<Point> &out);

void drawPoints(cv::Mat &image, std::vector<Point> &points, bool move, int index, double scale, int offset);

#endif