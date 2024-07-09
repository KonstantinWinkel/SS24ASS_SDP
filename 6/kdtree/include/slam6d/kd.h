/** @file 
 *  @brief Representation of the optimized k-d tree. 
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __KD_H__
#define __KD_H__

#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"

#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef MAX_OPENMP_NUM_THREADS
#define MAX_OPENMP_NUM_THREADS 1
#endif

#ifndef LEAF_SIZE
#define LEAF_SIZE 100
#endif

#include <vector>

#include <algorithm>

/** @file 
 *  @brief An optimized k-d tree implementation
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/kd.h"
#include "slam6d/globals.icc"          

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <algorithm>
using std::swap;
#include <cmath>
#include <cstring>

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtree : public SearchTree {

public:

  KDtree(double **pts, int n);

  void leafdraw(std::vector<double *>& interms, std::vector<double *>& leafes, int axis){
    // Find bbox
    double xmin = leaf.p[0][0], xmax = leaf.p[0][0];
    double ymin = leaf.p[0][1], ymax = leaf.p[0][1];
    double zmin = leaf.p[0][2], zmax = leaf.p[0][2];
    for (int i = 1; i < npts; i++) {
      xmin = min(xmin, leaf.p[i][0]);
      xmax = max(xmax, leaf.p[i][0]);
      ymin = min(ymin, leaf.p[i][1]);
      ymax = max(ymax, leaf.p[i][1]);
      zmin = min(zmin, leaf.p[i][2]);
      zmax = max(zmax, leaf.p[i][2]);
    }
    double *leafb;
    if(axis == 0)
      leafb = new double[5]{ymin,zmin, ymax, zmax,0};
    else if(axis == 1)
      leafb = new double[5]{xmin,zmin, xmax, zmax,0};
    else
      leafb = new double[5]{xmin,ymin, xmax, ymax,0};
    interms.push_back(leafb);
    for(int i = 0; i < npts; i++) {
      if(axis == 0)
        leafes.push_back(new double[]{leaf.p[i][1], leaf.p[i][2], 0});
      else if(axis == 1)
        leafes.push_back(new double[]{leaf.p[i][0], leaf.p[i][2], 0});
      else
        leafes.push_back(new double[]{leaf.p[i][0], leaf.p[i][1], 0});
    }
  }

  void intersectionsAtAxisValue(std::vector<double *>& interms, std::vector<double *>& leafes, int axis, double value){
    double minbound, maxbound;
    double other1min, other1max, other2min, other2max;
    if(axis == 0){
      maxbound = node.center[0] + node.dx*2;
      minbound = node.center[0] - node.dx*2;
      other1max = node.center[1] + node.dy*2;
      other1min = node.center[1] - node.dy*2;
      other2max = node.center[2] + node.dz*2;
      other2min = node.center[2] - node.dz*2;
    }
    else if(axis == 1){
      other1max = node.center[0] + node.dx*2;
      other1min = node.center[0] - node.dx*2;
      maxbound = node.center[1] + node.dy*2;
      minbound = node.center[1] - node.dy*2;
      other2max = node.center[2] + node.dz*2;
      other2min = node.center[2] - node.dz*2;
    }
    else{
      other1max = node.center[0] + node.dx*2;
      other1min = node.center[0] - node.dx*2;
      other2max = node.center[1] + node.dy*2;
      other2min = node.center[1] - node.dy*2;
      maxbound = node.center[2] + node.dz*2;
      minbound = node.center[2] - node.dz*2;
    }

      if(minbound < value && maxbound > value){
        double *nodeb = new double[5]{other1min, other2min, other1max, other2max,1};
        interms.push_back(nodeb);
        if(node.child1->npts  == 0){
          node.child1->intersectionsAtAxisValue(interms, leafes, axis, value);
        }
        else{
          node.child1->leafdraw(interms, leafes, axis);
        }
        if(node.child2->npts  == 0){
          node.child2->intersectionsAtAxisValue(interms, leafes, axis, value);
        }
        else{
          node.child2->leafdraw(interms, leafes, axis);
        }
      }
  }
  
  /**
   * destructor
   */
  virtual ~KDtree() {                
    if (!npts) {
#ifdef WITH_OPENMP_KD
      omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic)
#endif
      for (int i = 0; i < 2; i++) {
	   if (i == 0 && node.child1) delete node.child1; 
        if (i == 1 && node.child2) delete node.child2; 
	 }
    } else {
      if (leaf.p) delete [] leaf.p;
    }
  }

  double *FindClosest(double *_p, double maxdist2, int threadNum = 0);

private:
  /**
   * storing the parameters of the k-d tree, i.e., the current closest point,
   * the distance to the current closest point and the point itself.
   * These global variable are needed in this search.
   *
   * Padded in the parallel case.
   */
#ifdef _OPENMP
#ifdef __INTEL_COMPILER
  __declspec (align(16)) static KDParams params[MAX_OPENMP_NUM_THREADS];
#else
  static KDParams params[MAX_OPENMP_NUM_THREADS];
#endif //__INTEL_COMPILER
#else
  static KDParams params[MAX_OPENMP_NUM_THREADS];
#endif	

  /**
   * number of points. If this is 0: intermediate node. If nonzero: leaf.
   */
  int npts;
  
  /**
   * Cue the standard rant about anon unions but not structs in C++
   */
  union {
    /** 
     * in case of internal node... 
     */
    struct {	 
      double center[3]; ///< storing the center of the voxel (R^3)
      double dx,  ///< defining the voxel itself
	     dy,  ///< defining the voxel itself
	     dz,  ///< defining the voxel itself
	     r2;  ///< defining the voxel itself
      int splitaxis;   ///< defining the kind of splitaxis
      KDtree *child1;  ///< pointers to the childs
      KDtree *child2;  ///< pointers to the childs
    } node;
    /** 
     * in case of leaf node ... 
     */
    struct {
      /** 
       * store the value itself
       * Here we store just a pointer to the data
       */
      double **p;
    } leaf;
  };

  void _FindClosest(int threadNum);
};

#endif


