#include "helper.h"

double rad(const double deg) {
  return ( (2 * M_PI * deg) / 360 );
}

double dist2(Point p1, Point p2) {
  return sqr(p1.x - p2.x) + sqr(p1.y - p2.y) + sqr(p1.z - p2.z);
}

double sqr(double d) {
  return d*d;
}

void print(std::string filename, std::vector<Point> &points) {
  FILE *file = fopen(filename.c_str(), "w");
  for(unsigned int i = 0; i < points.size(); i++) {
    fprintf(file, "%lf %lf %lf\n", points[i].x, points[i].y, points[i].z);
  }
  fclose(file);
}

void transform(Point &p, const Matrix t) {
  double tmp = p.x*t(1,1) + p.y*t(1,2) + t(1,3);
  p.y = p.x*t(2,1) + p.y*t(2,2) + t(2,3);
  p.x = tmp;
}

double alignSVD(const vector<PtPair>& pairs, Matrix &alignfx, bool quiet)
{
  double error = 0;
  double sum = 0.0;
  
  //Get the center of the p1s and p2s
  double centroid_m[2];
  double centroid_d[2];
  centroid_m[0] = centroid_m[1] = centroid_d[0] = centroid_d[1] = 0;
  for(unsigned int i = 0; i <  pairs.size(); i++){
    centroid_m[0] += pairs[i].p1.x; 
    centroid_m[1] += pairs[i].p1.y; 
    centroid_d[0] += pairs[i].p2.x; 
    centroid_d[1] += pairs[i].p2.y; 
  }
  centroid_m[0] /= pairs.size();
  centroid_m[1] /= pairs.size();
  centroid_d[0] /= pairs.size();
  centroid_d[1] /= pairs.size();

  //we calculate the difference of each point to the center of its centroid
  double** m = new double*[pairs.size()];
  double** d = new double*[pairs.size()];

  for(unsigned int i = 0; i <  pairs.size(); i++){
    m[i] = new double[2];
    d[i] = new double[2];
    m[i][0] = pairs[i].p1.x - centroid_m[0];
    m[i][1] = pairs[i].p1.y - centroid_m[1];
    d[i][0] = pairs[i].p2.x - centroid_d[0];
    d[i][1] = pairs[i].p2.y - centroid_d[1];

    //we add up the squared distances for all points
    sum += sqr(pairs[i].p1.x - pairs[i].p2.x)
      + sqr(pairs[i].p1.y - pairs[i].p2.y);

  }

  //then we sqrt it and divide by the num of pairs
  error = sqrt(sum / (double)pairs.size());

  if (!quiet) {
    std::cout.setf(ios::basefield);
    std::cout << "SVD RMS point-to-point error = "
      << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
      << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
      << std::setw(10) << std::setprecision(7)
      << error
      << "  using " << std::setw(6) << (int)pairs.size() << " points" << endl;
  }

  Matrix H(2,2), R(2,2);
  //Initialize H with 0s , H is the covariance matrix
  for(int j = 0; j < 2; j++){
    for(int k = 0; k < 2; k++){
      H(j+1, k+1) = 0.0;     
    }
  }
  

  for(unsigned int i = 0; i < pairs.size(); i++){
    for(int j = 0; j < 2; j++){
      for(int k = 0; k < 2; k++){
        H(j+1, k+1) += d[i][j]*m[i][k]; //calculate the covariances and add them up
      }
    }
  }
    
  Matrix U(2,2);
  DiagonalMatrix Lamda(2);
  Matrix V(2,2);
  
  //Perfom SVD on H to get its eigenvectors
  SVD(H, Lamda, U, V);
   
  R = V*(U.t());

  //Negate the diagonal if the determinant of the Eigenvec-matrix is negative in hopes the determinat becomes positive
  // (cf. Arun, Huang, and Blostein: "Least-Squares Fitting of Two 3-D Point Sets")
  if(R.Determinant() < 0) {
    V(1, 2) = -V(1, 2);
    V(2, 2) = -V(2, 2);
    R = V*(U.t());

    if(R.Determinant() < 0) {
      // if we still failed, at least give a warning
      cerr << "Warning - icp6D_SVD::Align failed to compute a matching transformation - the returned solution is a reflection!" << endl;
    }
  }

  double translation[2];
  ColumnVector col_vec(2);
  for(int j = 0; j < 2; j++)
    col_vec(j+1) = centroid_d[j];
  //transform the centroids of the p2s by the eigenvectors
  ColumnVector r_time_colVec = ColumnVector(R*col_vec);
  //the translation is the centroid of the p1s - the transformed centroid of the p2s
  translation[0] = centroid_m[0] - r_time_colVec(1);
  translation[1] = centroid_m[1] - r_time_colVec(2);

  //Save out all data to param
  alignfx(1,1) = R(1,1);
  alignfx(1,2) = R(1,2);
  alignfx(1,3) = translation[0];
  alignfx(2,1) = R(2,1);
  alignfx(2,2) = R(2,2);
  alignfx(2,3) = translation[1];
  alignfx(3,1) = 0;
  alignfx(3,2) = 0;
  alignfx(3,3) = 1;

  //Cleanup
  for(unsigned int i = 0; i <  pairs.size(); i++) {
    delete [] m[i];
    delete [] d[i];
  }
  delete [] m;
  delete [] d;

  return error;
}

Matrix computeTransformation2(double theta, double dx, double dy) {
  Matrix t(3,3);
  double _theta = rad(theta);
  t(1,1) = cos(_theta);
  t(1,2) = -sin(_theta);
  t(1,3) = dx;
  t(2,1) = sin(_theta);
  t(2,2) = cos(_theta);
  t(2,3) = dy;
  t(3,1) = 0;
  t(3,2) = 0;
  t(3,3) = 1;
  return t;
}

void transformCloud(vector<Point> &points, Matrix &mat) {
  for(unsigned int i = 0; i < points.size(); i++) {
    transform(points[i], mat);
  }
}

void copyCloud(vector<Point> &in, vector<Point> &out) {
  for(unsigned int i = 0; i < in.size(); i++) {
    Point tmp;
    tmp.x = in[i].x;
    tmp.y = in[i].y;
    tmp.z = in[i].z;
    out.push_back(tmp);
  }
}

/*
 * Not the most beautiful way to visualize the results, but an idea.
 * No error handling implemented, yet!!!
 */
void drawPoints(cv::Mat &image, std::vector<Point> &points, bool move, int index, double scale, int offset) {
  unsigned char r,g,b;
  if(!move) 
    {r = 0; g = 0; b = 0;
  } else {
    r = 0; g = 10*index; b = 255 - 10*index;
  }

  for(unsigned int i = 0; i < points.size(); i++) {
    int x = (points[i].x * scale) + offset;
    int y = image.rows - (points[i].y*scale + offset);
    image.at<cv::Vec3b>(y, x)[0] = b;
    image.at<cv::Vec3b>(y, x)[1] = g;
    image.at<cv::Vec3b>(y, x)[2] = r;
  }
}
