#include "point.h"
#include "helper.h"
#include "generate.h"
#include <limits.h>
#include <iostream>
#include <opencv2/opencv.hpp>

void doTheThing(std::vector<Point> gridcloudM, std::vector<Point> gridcloudD, std::string name){
  cv::Size size(200,200); 
 cv::Mat_<cv::Vec3b> gridg = cv::Mat_<cv::Vec3b>(size);
  gridg.setTo(cv::Scalar(255,255,255));
  drawPoints(gridg, gridcloudM, false, 0, 1.0, 50);
  // draw data point cloud (iteration index 0)
  drawPoints(gridg, gridcloudD, true, 0, 1.0, 50);

  cv::imwrite(name+"/"+name+"-start.png", gridg);
  for(int i = 0; i < 50; i++){
    gridg.setTo(cv::Scalar(255,255,255));

    //brute force comparisons closest points approach
    std::vector<PtPair> pairs;
    for(auto& p1 : gridcloudM){
      Point* closest;
      double bestDist = 9999999999999999;
      for(auto& p2 : gridcloudD){
        double dist = sqr(p1.x-p2.x)+sqr(p1.y-p2.y);//z=0
        if(dist < bestDist){
          bestDist = dist;
          closest = &p2;
        }
      }
      PtPair pp;
      pp.p1 = p1;
      pp.p2 = *closest;
      pairs.push_back(pp);
    }

    //approach used later in the given file, just using the points at the same index.
    //yields better results but only works for this very specific case
    /*vector<PtPair> pairs;
    for (size_t i = 0; i < gridcloudM.size(); i++) {
      PtPair pair;
      pair.p1 = gridcloudM.at(i);
      pair.p2 = gridcloudD.at(i);
      pairs.push_back(pair);
    }*/
    NEWMAT::Matrix alignfx(3,3);
    double err = alignSVD(pairs, alignfx);

    transformCloud(gridcloudD, alignfx);
    drawPoints(gridg, gridcloudM, false, 0, 1.0, 50);
    drawPoints(gridg, gridcloudD, true, 0, 1.0, 50);
    cv::imwrite(name+"/"+name+"-iteration-"+std::to_string(i)+".png", gridg);

  }
}


int main(int argc, char * argv[])
{
  std::vector<Point> pointcloudM;
  std::vector<Point> pointcloudD;
  generateWave(pointcloudM); 
  copyCloud(pointcloudM, pointcloudD);
  Matrix mat;
  mat = computeTransformation2(20,10,10);
  transformCloud(pointcloudD,mat);
  doTheThing(pointcloudM, pointcloudD, "wave");

  std::vector<Point> pointcloud2M;
  std::vector<Point> pointcloud2D;
  generateWave(pointcloud2M); 
  copyCloud(pointcloud2M, pointcloud2D);
  mat = computeTransformation2(90,-50,40);
  transformCloud(pointcloud2D,mat);
  doTheThing(pointcloud2M, pointcloud2D, "wave2");

  
  
  std::vector<Point> gridcloudM;
  std::vector<Point> gridcloudD;
  generateGrid(gridcloudM);
  copyCloud(gridcloudM, gridcloudD);
  transformCloud(gridcloudD,mat);
  doTheThing(gridcloudM, gridcloudD, "grid");

  std::vector<Point> gridcloud2M;
  std::vector<Point> gridcloud2D;
  generateGrid(gridcloud2M);
  copyCloud(gridcloud2M, gridcloud2D);
  mat = computeTransformation2(45,-20,30);
  transformCloud(gridcloud2D,mat);
  doTheThing(gridcloud2M, gridcloud2D, "grid2");


  std::vector<Point> othercloudM;
  double radius = 50;
  for(double i = 0; i < 2*3.14159; i+=0.05) {
    Point p;
    p.x = radius * sin(i) + radius;
    p.y = radius * cos(i) + radius;
    p.z = 0;
    othercloudM.push_back(p);
  }
  std::vector<Point> othercloudD;
  copyCloud(othercloudM, othercloudD);
  transformCloud(othercloudD,mat);
  doTheThing(othercloudM, othercloudD, "other");


  std::vector<Point> othercloud2M;
  for(double i = 0; i < 2*3.14159; i+=0.05) {
    Point p;
    p.x = radius * sin(i) + radius;
    p.y = radius * cos(i) + radius;
    p.z = 0;
    othercloud2M.push_back(p);
  }
  std::vector<Point> othercloud2D;
  copyCloud(othercloud2M, othercloud2D);
  transformCloud(othercloud2D,mat);
  doTheThing(othercloud2M, othercloud2D, "other2");
 

}
