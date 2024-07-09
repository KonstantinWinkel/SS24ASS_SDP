#include "normals.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include "newmat/newmatap.h"
using namespace NEWMAT;

int main(int argc, char **argv) {

  std::string line;
  std::ifstream myfile(argv[1]);

  if (!myfile.is_open()) {
    std::cout << "Unable to open file" << std::endl;
    return 0;
  }
  std::vector<Point> allPoints;
  Point maxima{-100000, -10000, -10000};
  Point minima{100000, 100000, 100000};
  while (std::getline(myfile, line)) {
    std::stringstream ss(line);
    std::string s;
    Point p;
    int i = 0;
    while (std::getline(ss, s, ' ')) {
      switch (i) {
      case 0:
        p.x = -std::stof(s);
        if (p.x > maxima.x)
          maxima.x = p.x;
        if (p.x < minima.x)
          minima.x = p.x;
        break;
      case 1:
        p.y = -std::stof(s);
        if (p.y > maxima.y)
          maxima.y = p.y;
        if (p.y < minima.y)
          minima.y = p.y;
        break;
      case 2:
        p.z = std::stof(s);
        if (p.z > maxima.z)
          maxima.z = p.z;
        if (p.z < minima.z)
          minima.z = p.z;
        break;

      default:
        break;
      }
      i++;
    }
    allPoints.push_back(p);
  }
  myfile.close();

  float xspan = maxima.x - minima.x;
  float yspan = maxima.y - minima.y;

  std::vector<std::vector<std::vector<Point>>> grid11;
  grid11.resize(int(std::ceil(xspan)) + 1);
  for (auto &y : grid11) {
    y.resize(int(std::ceil(yspan)) + 1);
  }

  for (Point &p : allPoints) {
    int x = std::round(p.x - minima.x);
    int y = std::round(p.y - minima.y);
    grid11[x][y].push_back(p);
  }

  std::vector<std::vector<double *>> PCAgrid;
  PCAgrid.resize(int(std::ceil(xspan)) + 1);
  for (auto &y : PCAgrid) {
    y.resize(int(std::ceil(yspan)) + 1);
  }

  std::vector<std::vector<double>> flatgrid;
  flatgrid.resize(int(std::ceil(xspan)) + 1);
  for (auto &y : flatgrid) {
    y.resize(int(std::ceil(yspan)) + 1);
  }

  for (size_t x = 0; x < xspan + 1; x++) {
    for (size_t y = 0; y < yspan + 1; y++) {
      double *p = new double[4];

      DiagonalMatrix D; Matrix V;
      double res = calcPlane2(grid11[x][y], p, D, V);
      if (0 == res) {
        p[0] = 1;
        p[1] = 1;
        p[2] = 1;
        res = 1;
      }
      flatgrid[x][y] = res;
      PCAgrid[x][y] = p;
    }
  }

#define norma(Z) ((Z - minima.z) / zspan) * 255
#define fill(what, val) what.at<uchar>(x, y, 0) = val;

  cv::Size imgSize(
      yspan + 1,
      xspan + 2); // I have no idea why, but the second on does need to be +2

  cv::Mat normalImg(imgSize, CV_8UC3, cv::Scalar(0, 0, 255));
  cv::Mat normalsFlipImg(imgSize, CV_8UC3, cv::Scalar(0, 0, 255));
  cv::Mat flatImg(imgSize, CV_8U);

  for (size_t x = 0; x < xspan + 1; x++) {
    for (size_t y = 0; y < yspan + 1; y++) {
      double *v = PCAgrid[x][y];

      normalImg.at<cv::Vec3b>(x, y)[0] = (v[2] + 1) * 127.5;
      normalImg.at<cv::Vec3b>(x, y)[1] = (v[1] + 1) * 127.5;
      normalImg.at<cv::Vec3b>(x, y)[2] = (v[0] + 1) * 127.5;

      if (v[0] * v[0] + v[1] * v[1] + v[2] * v[2] >= 2.0) {
        normalsFlipImg.at<cv::Vec3b>(x, y)[0] = 255;
        normalsFlipImg.at<cv::Vec3b>(x, y)[1] = 255;
        normalsFlipImg.at<cv::Vec3b>(x, y)[2] = 255;
      } else {
        normalsFlipImg.at<cv::Vec3b>(x, y)[0] = (-v[2] + 1) * 127.5;
        normalsFlipImg.at<cv::Vec3b>(x, y)[1] = (-v[1] + 1) * 127.5;
        normalsFlipImg.at<cv::Vec3b>(x, y)[2] = (-v[0] + 1) * 127.5;
      }

      flatImg.at<uchar>(x, y, 0) = flatgrid[x][y] == 1 ? 255: flatgrid[x][y] * 1024;
    }
  }

  cv::imwrite("normals.png", normalImg);
  cv::imwrite("normalsFlip.png", normalsFlipImg);
  cv::imwrite("planarity.png", flatImg);
}