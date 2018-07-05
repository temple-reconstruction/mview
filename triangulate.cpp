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

  // projMatrs: 3x4 projection Matrix of camera1 and camera2
  // pixels:  2xN feature points
  // global: 4xN reconstructed points
  cv::triangulatePoints(projMatr1, projMatr2, pixel_left, pixel_right, global);
}

// void triangulate(const Rectified &rectified, Correspondence &correspondence)
// {
  
//   const auto &extrinsics = rectified.extrinsics;
//   const auto &intrinsics = rectified.intrinsics;
//   const auto &baseline_distance = rectified.baseline_distance;

//   const auto &pixel_left = correspondence.left;
//   const auto &pixel_right = correspondence.right;
//   const auto &cost = correspondence.cost;
//   const auto &global = correspondence.global;

//   const auto parallaxX = -(pixel_right.x - pixel_left.x);
//   const auto parallaxY = -(pixel_right.y - pixel_left.y);

//   global[2] = ((baseline_distance / parallaxX) * intrinsics[0]) * extrinsics;
//   global[0] = (pixel_left.x * (baseline_distance / parallaxX)) * extrinsics;
//   global[1] = (((pixel_left.y + pixel_right.y) / 2) * (baseline_distance / parallaxX))) * extrinsics;
// }
