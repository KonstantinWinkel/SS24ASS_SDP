#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

struct point {
  float x;
  float y;
  float z;
};

int main(int argc, char **argv) {

  std::string line;
  std::ifstream myfile(argv[1]);

  if (!myfile.is_open()) {
    std::cout << "Unable to open file";
    return 0;
  }
  std::vector<point> allPoints;
  point maxima{-100000, -10000, -10000};
  point minima{100000, 100000, 100000};
  while (std::getline(myfile, line)) {
    std::stringstream ss(line);
    std::string s;
    point p;
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
  float zspan = maxima.z - minima.z;
  
  std::vector<std::vector<std::vector<float>>> grid11;
  grid11.resize(int(std::ceil(xspan)) + 1);
  for (auto &y : grid11) {
    y.resize(int(std::ceil(yspan)) + 1);
  }

  std::vector<std::vector<std::vector<float>>> grid33;
  grid33.resize(int(std::ceil(xspan/3)) + 1);
  for (auto &y : grid33) {
    y.resize(int(std::ceil(yspan/3)) + 1);
  }

  for (point &p : allPoints) {
    int x = std::round(p.x - minima.x);
    int y = std::round(p.y - minima.y);
    grid11[x][y].push_back(p.z);

    int x3 = std::round((p.x - minima.x)/3);
    int y3 = std::round((p.y - minima.y)/3);
    grid33[x3][y3].push_back(p.z);
  }

#define norma(Z) ((Z - minima.z) / zspan) * 255
#define fill(what, val) what.at<uchar>(x, y, 0) = val;

  cv::Size imgSize(yspan+1,xspan+2); //I have no idea why, but the second on does need to be +2

  cv::Mat random(imgSize, CV_8U);
  cv::Mat stddev(imgSize, CV_8U);
  cv::Mat single(imgSize, CV_8U);
  cv::Mat first(imgSize, CV_8U);
  cv::Mat last(imgSize, CV_8U);
  cv::Mat difference(imgSize, CV_8U);
  

  for (size_t x = 0; x < xspan + 1; x++) {
    for (size_t y = 0; y < yspan + 1; y++) {
      std::vector<float>& v = grid11[x][y];
      if (v.size() == 0)
        continue;
      
      float sum = std::accumulate(std::begin(v), std::end(v), 0.0);
      float mean = sum / v.size();

      float accum = 0.0;
      std::for_each(std::begin(v), std::end(v),
                    [&](const float d) { accum += (d - mean) * (d - mean); });

      float stdev = sqrt(accum / (v.size() - 1));
      auto low_high = std::minmax_element(v.begin(), v.end());


      char r = norma(v[0]);
      fill(random, r);

      char sd = (stdev >= 1.0f) ? 0 : 255;
      fill(stddev, sd);

      char si = (stdev >= 1.0f) ? 0 : norma(mean);
      fill(single, si);

      char fir = (stdev >= 1.0f) ? norma(*low_high.second) : 0;
      fill(first, fir);

      char las = (stdev >= 1.0f) ? norma(*low_high.first) : 0;
      fill(last, las);

      char dif =
          (stdev >= 1.0f) ? norma(*low_high.second - *low_high.first) : 255;
      fill(difference, dif);
    }
  }
  cv::imwrite("../5/single.png", single);
  cv::imwrite("../5/random.png", random);
  cv::imwrite("../5/first.png", first);
  cv::imwrite("../5/last.png", last);
  cv::imwrite("../5/stddev.png", stddev);
  cv::imwrite("../5/difference.png", difference);


  cv::Size imgSize3(yspan/3+1, xspan/3+1);
  cv::Mat random3(imgSize3, CV_8U);
  cv::Mat stddev3(imgSize3, CV_8U);
  cv::Mat single3(imgSize3, CV_8U);
  cv::Mat first3(imgSize3, CV_8U);
  cv::Mat last3(imgSize3, CV_8U);
  cv::Mat difference3(imgSize3, CV_8U);
  

  for (size_t x = 0; x < xspan/3 ; x++) {
    for (size_t y = 0; y < yspan/3 ; y++) {
      std::vector<float>& v = grid33[x][y];
      if (v.size() == 0)
        continue;
      
      float sum = std::accumulate(std::begin(v), std::end(v), 0.0);
      float mean = sum / v.size();

      float accum = 0.0;
      std::for_each(std::begin(v), std::end(v),
                    [&](const float d) { accum += (d - mean) * (d - mean); });

      float stdev = sqrt(accum / (v.size() - 1));
      auto low_high = std::minmax_element(v.begin(), v.end());


      char r = norma(v[0]);
      fill(random3, r);

      char sd = (stdev >= 1.0f) ? 0 : 255;
      fill(stddev3, sd);

      char si = (stdev >= 1.0f) ? 0 : norma(mean);
      fill(single3, si);

      char fir = (stdev >= 1.0f) ? norma(*low_high.second) : 0;
      fill(first3, fir);

      char las = (stdev >= 1.0f) ? norma(*low_high.first) : 0;
      fill(last3, las);

      char dif =
          (stdev >= 1.0f) ? norma(*low_high.second - *low_high.first) : 255;
      fill(difference3, dif);
    }
  }
  cv::imwrite("../5/single3.png", single3);
  cv::imwrite("../5/random3.png", random3);
  cv::imwrite("../5/first3.png", first3);
  cv::imwrite("../5/last3.png", last3);
  cv::imwrite("../5/stddev3.png", stddev3);
  cv::imwrite("../5/difference3.png", difference3);
}