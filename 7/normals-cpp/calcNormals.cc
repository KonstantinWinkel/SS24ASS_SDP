#include <fstream>
#include <iostream>
#include <vector>
#include "normals.h"

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
    if (fscanf(file, "%lf %lf %lf",&tmp.x, &tmp.y, &tmp.z) == 3) {
      points.push_back(tmp);
    }
  }
  fclose(file);
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

  double plane[4];
  DiagonalMatrix D; Matrix V;
  double e = calcPlane2(points, plane, D, V);

  std::cout << "normal: "<<plane[0] << " " << plane[1] << " " << plane[2] << " " <<std::endl;
  std::cout << "flatness: " << e << std::endl;
  std::cout << "eigenvalues: " << D(1) << " " << D(2) << " " << D(3) << std::endl;
  std::cout << "eigenvector 1: " << V(1,1) << " " << V(2,1) << " " << V(3,1) << std::endl;
  std::cout << "eigenvector 2: " << V(1,2) << " " << V(2,2) << " " << V(3,2) << std::endl;
  std::cout << "eigenvector 3: " << V(1,3) << " " << V(2,3) << " " << V(3,3) << std::endl;



  return 0;
}
