#include "mview.h"

#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>

void triangulate(const Rectified &rectified, Correspondence &correspondence)
{
  const auto &pixel_left = correspondence.left;
  const auto &pixel_right = correspondence.right;
  const auto &global = correspondence.global;
  
  const auto &projMatr1 = rectified.R1;
  const auto &projMatr1 = rectified.R2;
  const auto &pixel_left = rectified.P1;
  const auto &pixel_right = rectified.P2;
  
  cv::triangulatePoints(projMatr1, projMatr2, pixel_left, pixel_right, global);
}

