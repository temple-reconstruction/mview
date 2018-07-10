#include "mview.h"

#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>

void triangulate(const Rectified &rectified, Disparity& disparity)
{
  std::vector<Correspondence> &correspondences = disparity.correspondences;

  cv::Mat Output4DPoints;
  cv::reprojectImageTo3D(disparity.disparity, Output4DPoints, rectified.Q);

  cv::Mat mask = disparity.disparity <= 0.;
  Output4DPoints.setTo(cv::Vec3f(), mask);
  
  /*j
  const float scale_y = rectified.pixel_left_gray.cols();
  const float scale_x = rectified.pixel_left_gray.rows();

  cv::Mat leftPoints0 (1, correspondences.size(), CV_32F);
  cv::Mat leftPoints1 (1, correspondences.size(), CV_32F);

  cv::Mat rightPoints0 (1, correspondences.size(), CV_32F);
  cv::Mat rightPoints1 (1, correspondences.size(), CV_32F);

  // 2xN corresponding points from correspondences
  for (int i = 0; i < correspondences.size(); i++)
  {
	const auto& corr = correspondences[i];
	leftPoints0.at<float>(0, i) = corr.left.x;
	leftPoints1.at<float>(0, i) = corr.left.y;

	rightPoints0.at<float>(0, i) = corr.right.x;
	rightPoints1.at<float>(0, i) = corr.right.y;
  }

  cv::Mat leftPoints[2] = {leftPoints0, leftPoints1};
  cv::Mat rightPoints[2] = {rightPoints0, rightPoints1};

  cv::Mat leftPointsMat;
  cv::Mat rightPointsMat;

  cv::merge(leftPoints, 2, leftPointsMat);
  cv::merge(rightPoints, 2, rightPointsMat);
  
  const auto &left_projection_matrix = rectified.P1;
  const auto &right_projection_matrix = rectified.P2;

  cv::undistortPoints(leftPointsMat, leftPointsMat, left_projection_matrix.colRange(0, 3), {});
  cv::undistortPoints(rightPointsMat, rightPointsMat, right_projection_matrix.colRange(0, 3), {});

  cv::split(leftPointsMat, leftPoints);
  cv::split(rightPointsMat, rightPoints);

  cv::vconcat(leftPoints, 2, leftPointsMat);
  cv::vconcat(rightPoints, 2, rightPointsMat);

  cv::triangulatePoints(left_projection_matrix, right_projection_matrix,
                        leftPointsMat, rightPointsMat,
                        Output4DPoints);

  
  for(int i = 0; i < 4; i++) {
    // Iteration order is important here
	Output4DPoints.row(i) /= Output4DPoints.row(3);
  }
 

  //TODO assert if size of correspondences is equal to number of Output4DPoints generated
  */
  assert(correspondences.size() == disparity.disparity.cols * disparity.disparity.rows);

  cv::Mat extrinsics1;
  cv::eigen2cv(rectified.extrinsics_left, extrinsics1);

  cv::Mat global_parts[3];
  cv::split(Output4DPoints, global_parts);
  for(int i = 0; i < 3; i++)
    global_parts[i] = global_parts[i].reshape(1, 1);

  cv::Mat globals;
  cv::vconcat(global_parts, 3, globals);

  cv::Mat local_rotation(3, 3, CV_32F);
  cv::Mat global_rotation(3, 3, CV_32F);
  cv::Mat shift(3, 1, CV_32F);
  rectified.R1.convertTo(local_rotation, CV_32F);
  local_rotation = local_rotation.t();

  global_rotation = extrinsics1(cv::Range(0, 3), cv::Range(0, 3)).t();
  shift = -global_rotation * extrinsics1.col(3).rowRange(0, 3);

  globals = local_rotation * globals;
  globals = global_rotation * globals;
  for(int i = 0; i < globals.cols; i++)
    globals.col(i) += shift;

  for (int i = 0; i < globals.cols; i++){
	cv::cv2eigen(globals.col(i).rowRange(0, 3), correspondences[i].global);
  }
}
