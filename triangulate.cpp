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
  std::vector<cv::Point3d> points_3d;
  
  // 2xN corresponding points from correspondences
  for (int i = 0; i < correspondences.size(); i++)
  {
    left_projection_points.push_back(correspondences[i].left.x, correspondences[i].left.y);
    right_projection_points.push_back(correspondences[i].right.x, correspondences[i].right.y);
  }
  
  const auto &left_projection_matrix = rectified.P1;
  const auto &right_projection_matrix = rectified.P2;

  cv::triangulatePoints(left_projection_matrix, right_projection_matrix,
                        left_projection_points, right_projection_points,
                        Output4DPoints);

  for (int i = 0; i < Output4DPoints.cols; i++)
  {
    cv::Mat point = Output4DPoints.col(i);
    point /= point.at<float>(3, 0);
    cv::Point3d p(
        point.at<float>(0, 0),
        point.at<float>(1, 0),
        point.at<float>(2, 0));

    points_3d.push_back(p);
  }

  //TODO assert if size of correspondences is equal to number of Output4DPoints generated

  for (int i = 0; i < correspondences.size(); i++){
    cv::cv2eigen(points_3d, correspondences[i].global);
  }

}
