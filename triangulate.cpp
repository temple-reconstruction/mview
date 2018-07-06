#include "mview.h"

#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>

void triangulate(const Rectified &rectified, std::vector<Correspondence> &correspondences)
{
  
  cv::Mat Output4DPoints;
  std::vector<cv::Point2f> left_projection_points, right_projection_points;
  
  // 2xN corresponding points from correspondences
  for (int i = 0; i < correspondences.size(); i++)
  {
    left_projection_points.emplace_back(correspondences[i].left.x, correspondences[i].left.y);
    right_projection_points.emplace_back(correspondences[i].right.x, correspondences[i].right.y);
  }
  
  const auto &left_projection_matrix = rectified.P1;
  const auto &right_projection_matrix = rectified.P2;

  cv::triangulatePoints(left_projection_matrix, right_projection_matrix,
                        left_projection_points, right_projection_points,
                        Output4DPoints);

  for(int i = 0; i < 4; i++) {
    // Iteration order is important here
	Output4DPoints.row(i) /= Output4DPoints.row(3);
  }

  //TODO assert if size of correspondences is equal to number of Output4DPoints generated

  for (int i = 0; i < correspondences.size(); i++){
	cv::cv2eigen(Output4DPoints.col(i).rowRange(0, 3), correspondences[i].global);
  }

}
