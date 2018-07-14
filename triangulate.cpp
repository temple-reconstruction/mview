#include "mview.h"

#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>

Triangulated triangulate(const Rectified &rectified, const Disparity& disparity)
{
  cv::Mat reconstructed_points;
  cv::reprojectImageTo3D(disparity.disparity, reconstructed_points, rectified.Q);

  cv::Mat mask = disparity.disparity <= 5.;
  reconstructed_points.setTo(cv::Vec3f(), mask);

  cv::Mat point_parts[3];
  cv::split(reconstructed_points, point_parts);
  for(int i = 0; i < 3; i++)
    point_parts[i] = point_parts[i].reshape(1, 1);

  cv::Mat points;
  cv::vconcat(point_parts, 3, points);

  Eigen::Matrix3f local_rotation;
  cv::cv2eigen(rectified.R1, local_rotation);
  Eigen::Matrix4f local_extrinsics;
  local_extrinsics.setIdentity();
  local_extrinsics.block<3, 3>(0, 0) = local_rotation;

  Eigen::Matrix4f global_extrinsics = rectified.extrinsics_left;
  Eigen::Matrix4f extrinsics = local_extrinsics * global_extrinsics;

  Eigen::Matrix3f intrinsics;
  cv::cv2eigen(rectified.P1.colRange(0, 3), intrinsics);

  std::cout << extrinsics << std::endl;

  PointImage point_image(disparity.disparity.rows, disparity.disparity.cols);
  ColourImage colour_image(disparity.disparity.rows, disparity.disparity.cols);

  for (int i = 0; i < points.cols; i++){
	int x = i/disparity.disparity.cols;
	int y = i%disparity.disparity.cols;

	Eigen::Vector3f point;
	cv::cv2eigen(points.col(i).rowRange(0, 3), point);
	point_image(x, y) = point;
	colour_image(x, y).block<3, 1>(0, 0) = rectified.pixel_left_rgb(x, y);
  }

  return { point_image, colour_image, extrinsics, intrinsics };
}
