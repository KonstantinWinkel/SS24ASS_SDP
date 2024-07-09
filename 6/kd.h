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
///The rest is the given KDTree implementation
}
