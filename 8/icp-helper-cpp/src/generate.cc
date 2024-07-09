#include "generate.h"
#include "helper.h"

void generateGrid(std::vector<Point> &points) {
  for(double x = 0; x <= 100; x +=10.0) {
    for(double y = 0; y <= 100; y +=10.0) {
      Point tmp;
      tmp.x = x;
      tmp.y = y;
      tmp.z = 0;
      points.push_back(tmp);
    }
  }
}

void generateWave(std::vector<Point> &points) {
  for(double x = 0; x <= 100; x +=1.0) {
    Point tmp;
    tmp.x = x;
    tmp.y = 20.0*sin(0.1*x);
    tmp.z = 0;
    points.push_back(tmp);
  }
}