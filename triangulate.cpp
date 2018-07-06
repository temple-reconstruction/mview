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

  cv::Mat coordinates[3];
  cv::split(Output4DPoints, coordinates);
  assert(correspondences.size() == disparity.disparity.cols * disparity.disparity.rows);

  for (int i = 0; i < correspondences.size(); i++){
	int col = i%disparity.disparity.cols;
	int row = i/disparity.disparity.cols;
	correspondences[i].global = {coordinates[0].at<float>(row, col), coordinates[1].at<float>(row, col), coordinates[2].at<float>(row, col)};
	// cv::cv2eigen(Output4DPoints.col(i).rowRange(0, 3), correspondences[i].global);
  }
}
