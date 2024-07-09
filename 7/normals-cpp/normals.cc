/*
 * Normal calculation implementation
 *
 * Copyright (C) Dorit Borrmann, Jan Elseberg, Kai Lingemann, Andreas Nuechter, Remus Dumitru
 *
 * Released under the GPL version 3.
 *
 */

#include "newmat/newmatap.h"
using namespace NEWMAT;
#include "normals.h"
#include <math.h>
#include <time.h>
#include <limits>
#include <errno.h>
#include <vector> 
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#include <direct.h>
#else
#include <dlfcn.h>
#endif 
#include <sys/stat.h>


/**
  * Given a set of points this function will calculate the best fit plane
  * @param 
  * @param 
  */
double calcPlane(std::vector<Point> &ppoints, double plane[4]) {
  SymmetricMatrix A(3);
  A = 0;
  int n;
  n = ppoints.size();
  if(n < 3) return 0;
  double cx, cy, cz;
  cx = 0.0;
  cy = 0.0;
  cz = 0.0;
  
  for (int i = 0; i < n; i++) {
    Point p = ppoints[i];
    cx += p.x;
    cy += p.y;
    cz += p.z;
  }
  cx /= n;
  cy /= n;
  cz /= n;
  
  for (int i = 0; i < n; i++) {
    Point p = ppoints[i];
    A(1, 1) += (p.x - cx)*(p.x - cx);
    A(2, 2) += (p.y - cy)*(p.y - cy);
    A(3, 3) += (p.z - cz)*(p.z - cz);
    A(1, 2) += (p.x - cx)*(p.y - cy);
    A(1, 3) += (p.x - cx)*(p.z - cz);
    A(2, 3) += (p.y - cy)*(p.z - cz);
  }
  
  DiagonalMatrix D;
  Matrix V;
  try {
    Jacobi(A,D,V);
  } catch (ConvergenceException) {
    std::cout << "couldn't find plane..." << std::endl;
    return 0;
  }
  
  int index;
  D.MinimumAbsoluteValue1(index);
     
  plane[0] = V(1,index);
  plane[1] = V(2,index);
  plane[2] = V(3,index);
  plane[3] = plane[0]*cx + plane[1]*cy + plane[2]*cz;
  
  double sum = 0.0;
  for(int i = 1; i < 4; i++) {
    sum += D(i);
  }
  sum = D(1)/sum;
  return sum;
}

double calcPlane2(std::vector<Point> &ppoints, double plane[4], DiagonalMatrix& D,  Matrix& V) {
  SymmetricMatrix A(3);
  A = 0;
  int n;
  n = ppoints.size();
  if(n < 3) return 0;
  double cx, cy, cz;
  cx = 0.0;
  cy = 0.0;
  cz = 0.0;
  
  for (int i = 0; i < n; i++) {
    Point p = ppoints[i];
    cx += p.x;
    cy += p.y;
    cz += p.z;
  }
  cx /= n;
  cy /= n;
  cz /= n;
  
  for (int i = 0; i < n; i++) {
    Point p = ppoints[i];
    A(1, 1) += (p.x - cx)*(p.x - cx);
    A(2, 2) += (p.y - cy)*(p.y - cy);
    A(3, 3) += (p.z - cz)*(p.z - cz);
    A(1, 2) += (p.x - cx)*(p.y - cy);
    A(1, 3) += (p.x - cx)*(p.z - cz);
    A(2, 3) += (p.y - cy)*(p.z - cz);
  }
  
  try {
    Jacobi(A,D,V);
  } catch (ConvergenceException) {
    std::cout << "couldn't find plane..." << std::endl;
    return 0;
  }

  int index;
  D.MinimumAbsoluteValue1(index);
  plane[0] = V(1,index);
  plane[1] = V(2,index);
  plane[2] = V(3,index);
  plane[3] = plane[0]*cx + plane[1]*cy + plane[2]*cz;
  
  return D(1) / D(2);
}