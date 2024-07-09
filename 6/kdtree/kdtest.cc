#include <fstream>
#include <iostream>
#include <vector>
#include <limits>
#include <opencv2/opencv.hpp>
#include "slam6d/kd.h"

struct Point {
  double x;
  double y;
  double z;
};

void readPointcloud (const char *filename, std::vector<Point> &points) {
  FILE *file = fopen(filename, "r"); 
  if (file == NULL) {
    perror ("Error opening file"); 
    return;
  } else {
    printf("Opening file %s\n", filename);
  }

  while (!feof(file)) {
    Point tmp;
    fscanf(file, "%lf %lf %lf",&tmp.x, &tmp.y, &tmp.z);
    points.push_back(tmp);
  }
  fclose(file);  
}

void convert(std::vector<Point> &in, double ** &out) {
  out = new double*[in.size()];
  for (int i = 0; i < in.size(); i++) {
    out[i] = new double[3];
    out[i][0] = in[i].x;
    out[i][1] = in[i].y;
    out[i][2] = in[i].z;
  }
}

void calcMinMax(std::vector<Point> &points, Point &min, Point &max) {
  for(std::vector<Point>::iterator it = points.begin(); it != points.end(); ++it) {
    if((*it).x < min.x) min.x = (*it).x;
    if((*it).y < min.y) min.y = (*it).y;
    if((*it).z < min.z) min.z = (*it).z;
    if((*it).x > max.x) max.x = (*it).x;
    if((*it).y > max.y) max.y = (*it).y;
    if((*it).z > max.z) max.z = (*it).z;
  }
}

void calcSize(cv::Size &size, Point min, Point max, double factor=1) {
  size.height = ceil(factor*(max.y - min.y));
  size.width = ceil(factor*(max.x - min.x));
}

void drawKDTree(std::vector<double *> &points, std::vector<double *> &corners, cv::Mat &image, cv::Size size, Point min, Point max, double factor=1) {
  int count = 0;
  cv::Scalar linecol(255,0,0);
  cv::Scalar leafcol(0,255,0);
  for (std::vector<double *>::iterator it = corners.begin(); it != corners.end(); ++it) {   
  
    if ((*it)[4]) {
      float p0x= (*it)[0];
      float p0y = (*it)[1];
      float p1x= (*it)[0];
      float p1y = (*it)[3];
      float p2x= (*it)[2];
      float p2y = (*it)[3];
      float p3x= (*it)[2];
      float p3y = (*it)[1];
      p0x = (unsigned int)(floor(factor*(p0x - min.x))); 
      p0y = size.height - (unsigned int)(floor(factor*(p0y - min.y))) - 1;
      p1x = (unsigned int)(floor(factor*(p1x - min.x))); 
      p1y = size.height - (unsigned int)(floor(factor*(p1y - min.y))) - 1;
      p2x = (unsigned int)(floor(factor*(p2x - min.x))); 
      p2y = size.height - (unsigned int)(floor(factor*(p2y - min.y))) - 1;
      p3x = (unsigned int)(floor(factor*(p3x - min.x))); 
      p3y = size.height - (unsigned int)(floor(factor*(p3y - min.y))) - 1;
      cv::Point p0(p0x,p0y);
      cv::Point p1(p1x,p1y);
      cv::Point p2(p2x,p2y);
      cv::Point p3(p3x,p3y);
    
      cv::line(image,p0,p1,linecol,1);
      cv::line(image,p1,p2,linecol,1);
      cv::line(image,p2,p3,linecol,1);
      cv::line(image,p3,p0,linecol,1);
    }
  }
  
  for (std::vector<double *>::iterator it = corners.begin(); it != corners.end(); ++it) {   
    if (!(*it)[4]) {
      float p0x= (*it)[0];
      float p0y = (*it)[1];
      float p1x= (*it)[0];
      float p1y = (*it)[3];
      float p2x= (*it)[2];
      float p2y = (*it)[3];
      float p3x= (*it)[2];
      float p3y = (*it)[1];
   
      p0x = (unsigned int)(floor(factor*(p0x - min.x))); 
      p0y = size.height - (unsigned int)(floor(factor*(p0y - min.y))) - 1;
      p1x = (unsigned int)(floor(factor*(p1x - min.x))); 
      p1y = size.height - (unsigned int)(floor(factor*(p1y - min.y))) - 1;
      p2x = (unsigned int)(floor(factor*(p2x - min.x))); 
      p2y = size.height - (unsigned int)(floor(factor*(p2y - min.y))) - 1;
      p3x = (unsigned int)(floor(factor*(p3x - min.x))); 
      p3y = size.height - (unsigned int)(floor(factor*(p3y - min.y))) - 1;
      cv::Point p0(p0x,p0y);
      cv::Point p1(p1x,p1y);
      cv::Point p2(p2x,p2y);
      cv::Point p3(p3x,p3y);
    
      count++;
      cv::line(image,p0,p1,leafcol,1);
      cv::line(image,p1,p2,leafcol,1);
      cv::line(image,p2,p3,leafcol,1);
      cv::line(image,p3,p0,leafcol,1);
    }
  }
   
  cv::Scalar pntcol(255,0,0);
  for (std::vector<double *>::iterator it = points.begin(); it != points.end(); ++it) {
    cv::Point curr((*it)[0],(*it)[1]);
    unsigned int y = size.height - (unsigned int)(floor(factor*(float)((*it)[1] - min.y))) - 1; 
    unsigned int x = (unsigned int)(floor(factor*(float)((*it)[0] - min.x)));
    image.at<cv::Vec3b>(y, x)[0] = 0;
    image.at<cv::Vec3b>(y, x)[1] = 0;
    image.at<cv::Vec3b>(y, x)[2] = 255;
  }
  
  std::cout << count << " leaf nodes drawn" << std::endl;
}

int main(int argc, char* argv[]) {
  const double factor = 10;
  
  std::vector<Point> points;
  
  if (argc > 1) { 
    readPointcloud(argv[1], points);
  }
  
  std::cout << points.size() << " points read" << std::endl;
  
  if (points.size() < 1) {
    return -1;
  }
  
  Point min = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  Point max = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
 
  calcMinMax(points, min, max);
  std::cout << "Minimum: " << min.x << " " << min.y << " " << min.z << std::endl;
  std::cout << "Maximum: " << max.x << " " << max.y << " " << max.z << std::endl;
  
  cv::Size size;
  calcSize(size, min, max, factor);
  std::cout << "Image size: " << size.width << "x" << size.height << std::endl;
  
  cv::Mat_<cv::Vec3b> kdImage = cv::Mat_<cv::Vec3b>(size);
  kdImage.setTo(cv::Scalar(255,255,255));
 
  double **pts;
  size_t nrPoints = points.size();
  convert(points, pts);

  // create k-d tree
  KDtree *kd = new KDtree(pts, nrPoints);
  
  std::vector<double *> interms;
  std::vector<double *> leafs;
  kd->intersectionsAtAxisValue(interms,leafs, 2,20);
  
  /*
  // DUMMY DATA
  double cleaf[5] = {-5, -5, 5, 5, 0}; // xmin, ymin, xmax, ymax, type
  corners.push_back(cleaf);
  double cnode1[5] = {-20, -20, 20, 20, 1}; // xmin, ymin, xmax, ymax, type
  corners.push_back(cnode1);
  double cnode2[5] = {20, -10, 40, 30, 1}; // xmin, ymin, xmax, ymax, type
  corners.push_back(cnode2);
  double p[3] = {0, 0, 0};
  levelpoints.push_back(p);
  // DUMMY DATA
  */

  drawKDTree(leafs, interms, kdImage, size, min, max, factor);

  cv::imwrite("kdtree.png", kdImage);
  
  return 0;
}
