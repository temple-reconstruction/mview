//
// Created by wang yu on 2018/6/20.
//
#include "mview.h"
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/core/eigen.hpp>

cv::Mat convertRgbToOpenCV(const RgbImage& rgb) {
	GrayImage r = rgb.unaryExpr([](Eigen::Vector3f rgb) { return rgb[0]; });
	GrayImage g = rgb.unaryExpr([](Eigen::Vector3f rgb) { return rgb[1]; });
	GrayImage b = rgb.unaryExpr([](Eigen::Vector3f rgb) { return rgb[2]; });

	cv::Mat r_mat;
    cv::eigen2cv(r,r_mat);
	cv::Mat g_mat;
    cv::eigen2cv(g,g_mat);
	cv::Mat b_mat;
    cv::eigen2cv(b,b_mat);

	cv::Mat arr[3] { r_mat, g_mat, b_mat };
	cv::Mat rgb_mat;
	cv::merge(arr, 3, rgb_mat);
	return rgb_mat;
}

RgbImage convertOpenCVToRgb(const cv::Mat imageMat){
    cv::Mat rgb[3];
    cv::split(imageMat, rgb);

	int width = imageMat.cols;
	int height = imageMat.rows;
    GrayImage r(height, width), g(height, width), b(height, width);

    cv::cv2eigen(rgb[0], r);
    cv::cv2eigen(rgb[1], g);
    cv::cv2eigen(rgb[2], b);

    RgbImage rgbImage (g.rows(), g.cols());

    for(int row=0;row<g.rows();row++){
        for(int col=0;col<g.cols();col++){
            rgbImage(row,col)<<r(row,col),g(row,col),b(row,col);
        }
    }

    return rgbImage;
}


void remap_rgb(cv::Mat& rgb_image, cv::Mat map1, cv::Mat map2) {
	cv::Mat rgbs[3];
	cv::split(rgb_image, rgbs);
  
	cv::Mat rgbs_out[3];
	for(int i = 0; i < 3; i++)
		cv::remap(rgbs[i], rgbs_out[i], map1, map2, cv::INTER_NEAREST, cv::BORDER_CONSTANT);

	cv::merge(rgbs_out, 3, rgb_image);
}

auto rectify(const Image& left, const Image& right) -> Rectified{
    //load data
    GrayImage left_gray_pixels=left.gray_pixels;
    RgbImage left_rgb_pixels=left.rgb_pixels;

    GrayImage right_gray_pixels=right.gray_pixels;
    RgbImage right_rgb_pixels=right.rgb_pixels;

	Eigen::Matrix4f left_to_right = right.extrinsics * left.extrinsics.inverse();
	for(int i = 0; i < 3; i++)
		assert(left_to_right(3, i) == 0);
	assert(left_to_right(3, 3) == 1.0);

    //rotation and transformation from left camera to right camera
    Eigen::Matrix3f R = left_to_right.block<3, 3>(0, 0);
    Eigen::Vector3f T = R.transpose() * left_to_right.block<3, 1>(0, 3);

    //convert eigen matrix to mat
    cv::Mat left_gray_mat;
    cv::eigen2cv(left_gray_pixels,left_gray_mat);
    cv::Mat left_rgb_mat = convertRgbToOpenCV(left_rgb_pixels);
    cv::Mat right_gray_mat;
    cv::eigen2cv(right_gray_pixels,right_gray_mat);
    cv::Mat right_rgb_mat = convertRgbToOpenCV(right_rgb_pixels);
    cv::Mat left_intrinsics_mat, right_intrinsics_mat, R_mat, T_mat;

    cv::eigen2cv(left.intrinsics,left_intrinsics_mat);
    cv::eigen2cv(right.intrinsics,right_intrinsics_mat);

    cv::eigen2cv(R,R_mat);
    cv::eigen2cv(T,T_mat);

	R_mat.assignTo(R_mat, CV_64F);
	T_mat.assignTo(T_mat, CV_64F);

    cv::Size imageSize = left_gray_mat.size();
	cv::Size targetSize = imageSize; // (left_gray_mat.cols/4., left_gray_mat.rows/4.);
    cv::Mat R1,R2,P1,P2,Q;
    cv::stereoRectify(left_intrinsics_mat,{},right_intrinsics_mat,{},imageSize,R_mat,T_mat,R1,R2,P1,P2,Q,
			cv::CALIB_ZERO_DISPARITY,
		   	1,targetSize,0,0);

    cv::Mat map1,map2, left_gray_out;
    cv::initUndistortRectifyMap(left_intrinsics_mat,{},R1,P1,targetSize,CV_32FC1,map1,map2);
    cv::remap(left_gray_mat,left_gray_out,map1,map2,cv::INTER_NEAREST,cv::BORDER_CONSTANT);
	left_gray_mat = left_gray_out;
	remap_rgb(left_rgb_mat, map1, map2);

    cv::Mat map3,map4, right_gray_out;
    cv::initUndistortRectifyMap(right_intrinsics_mat,{},R2,P2,targetSize,CV_32FC1,map3,map4);
    cv::remap(right_gray_mat,right_gray_out,map3,map4,cv::INTER_NEAREST,cv::BORDER_CONSTANT);
	right_gray_mat = right_gray_out;
	remap_rgb(right_rgb_mat, map3, map4);

	std::cout << "Debugging rectification results\n";
	std::cout << "R1 " << R1 << "\nR2 " << R2 << "\nP1 " << P1 << "\nP2" << P2 << "\nQ " << Q << std::endl;

	Rectified rectified;

    //convert mat to eigen matrix
	rectified.pixel_left_gray = GrayImage(left_gray_mat.rows, left_gray_mat.cols);
    cv::cv2eigen(left_gray_mat, rectified.pixel_left_gray);
	rectified.pixel_left_rgb = convertOpenCVToRgb(left_rgb_mat);

	rectified.pixel_right_gray = GrayImage (right_gray_mat.rows, right_gray_mat.cols);
    cv::cv2eigen(right_gray_mat, rectified.pixel_right_gray);
	rectified.pixel_right_rgb = convertOpenCVToRgb(right_rgb_mat);

	rectified.extrinsics_left = left.extrinsics;
	rectified.extrinsics_right = right.extrinsics;

	rectified.R1 = R1;
	rectified.R2 = R2;
	rectified.P1 = P1;
	rectified.P2 = P2;
	rectified.Q = Q;

    return rectified;
}

