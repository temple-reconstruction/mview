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

RgbImage convertOpenCVToRgb(const cv::Mat rgbMat){
    std::vector<cv::Mat> rgb;
    cv::split(rgbMat,rgb);
    GrayImage r,g,b;
    cv::cv2eigen(rgb[0],r);
    cv::cv2eigen(rgb[1],g);
    cv::cv2eigen(rgb[2],b);
//    std::cout<<g.rows()<<std::endl;
    RgbImage rgbImage (g.rows(), g.cols());
// Eigen::Matrix<Eigen::Vector3f,480,640> rgbImage;
//    rgbImage.setZero();
    for(int row=0;row<480;row++){
        for(int col=0;col<640;col++){
            rgbImage(row,col)<<r(row,col),g(row,col),b(row,col);
        }
    }
//    std::cout<<rgbImage.rows()<<std::endl;
    return rgbImage;
}


void remap_rgb(cv::Mat rgb_image, cv::Mat map1, cv::Mat map2) {
//	std::cout << rgb_image.channels() << " " << rgb_image.type() << " " << rgb_image.size() << "\n";

	cv::Mat rgbs[3];
	cv::split(rgb_image, rgbs);

	for(int i = 0; i < 3; i++)
		cv::remap(rgbs[i], rgbs[i], map1, map2, cv::INTER_NEAREST, cv::BORDER_CONSTANT);

	cv::merge(rgbs, 3, rgb_image);
}

auto rectify(const Image& left, const Image& right) -> Rectified{
    //load data
    GrayImage left_gray_pixels=left.gray_pixels;
    RgbImage left_rgb_pixels=left.rgb_pixels;
    Eigen::Matrix3f left_intrinsics=left.intrinsics;
    Eigen::Matrix4f left_extrinsics=left.extrinsics;

    GrayImage right_gray_pixels=right.gray_pixels;
    RgbImage right_rgb_pixels=right.rgb_pixels;
    Eigen::Matrix3f right_intrinsics=right.intrinsics;
    Eigen::Matrix4f right_extrinsics=right.extrinsics;


    Eigen::Vector3f t_l=Eigen::Vector3f(left_extrinsics(0,3),left_extrinsics(1,3),left_extrinsics(2,3));
    Eigen::Vector3f t_r=Eigen::Vector3f(right_extrinsics(0,3),right_extrinsics(1,3),right_extrinsics(2,3));
    Eigen::Matrix3f R_l=left_extrinsics.block<3,3>(0,0);
    Eigen::Matrix3f R_r=right_extrinsics.block<3,3>(0,0);

    //rotation and transformation from left image to right image
    Eigen::Matrix3f R=R_r*(R_l.transpose());
    Eigen::Vector3f T=t_r-R*t_l;
//
//    //compute Rotation matrix of rectification
//    Eigen::Vector3f e1=T/T.

    //convert eigen matrix to mat
    cv::Mat left_gray_mat;
    cv::eigen2cv(left_gray_pixels,left_gray_mat);
    cv::Mat left_rgb_mat = convertRgbToOpenCV(left_rgb_pixels);
    cv::Mat right_gray_mat;
    cv::eigen2cv(right_gray_pixels,right_gray_mat);
    cv::Mat right_rgb_mat = convertRgbToOpenCV(right_rgb_pixels);
    cv::Mat left_intrinsics_mat,left_extrinsics_mat,right_intrinsics_mat,right_extrinsics_mat,R_mat,T_mat;

    cv::eigen2cv(left_intrinsics,left_intrinsics_mat);
    cv::eigen2cv(left_extrinsics,left_extrinsics_mat);
    cv::eigen2cv(right_extrinsics,right_extrinsics_mat);
    cv::eigen2cv(right_intrinsics,right_intrinsics_mat);
    cv::eigen2cv(R,R_mat);
    cv::eigen2cv(T,T_mat);

	R_mat.assignTo(R_mat, CV_64F);
	T_mat.assignTo(T_mat, CV_64F);

//    cv::MatSize size=left_gray_mat.size;

    cv::Size imageSize=left_gray_mat.size();
    cv::Mat R1,R2,P1,P2,Q;
    std::vector<int > disCoeff(4,1);
//    cv::stereoRectify(left_intrinsics_mat,disCoeff,right_intrinsics_mat,disCoeff,size,R1,R2,P1,P2,Q,);
    cv::stereoRectify(left_intrinsics_mat,{},right_intrinsics_mat,{},imageSize,R_mat,T_mat,R1,R2,P1,P2,Q,0,-1,imageSize,0,0);

    cv::Mat map1,map2;
    cv::initUndistortRectifyMap(left_intrinsics_mat,disCoeff,R1,P1,imageSize,CV_32FC1,map1,map2);
    cv::remap(left_gray_mat,left_gray_mat,map1,map2,cv::INTER_NEAREST,cv::BORDER_CONSTANT);
//	std::cout << left_gray_mat.channels() << " " << left_gray_mat.type() << " " << left_gray_mat.size() << "\n";
//	std::cout << "Remapped gray\n";
	remap_rgb(left_rgb_mat, map1, map2);
//	std::cout << "Remapped rgb\n";

    cv::Mat map3,map4;
    cv::initUndistortRectifyMap(right_intrinsics_mat,disCoeff,R2,P2,imageSize,CV_32FC1,map3,map4);
    cv::remap(right_gray_mat,right_gray_mat,map3,map4,cv::INTER_NEAREST,cv::BORDER_CONSTANT);
//	std::cout << "Remapped gray\n";
	remap_rgb(right_rgb_mat, map3, map4);
//	std::cout << "Remapped rgb\n";

    //convert mat to eigen matrix
//    struct Rectified {
//        GrayImage pixel_left_gray;
//        GrayImage pixel_right_gray;
//
//        RgbImage pixel_left_rgb;
//        RgbImage pixel_right_rgb;
//
//        float baseline_distance;
//        Eigen::Matrix4f extrinsics;
//    };
    cv::cv2eigen(left_gray_mat,left_gray_pixels);
//    cv::cv2eigen(left_rgb_mat,left_rgb_pixels);
    left_rgb_pixels=convertOpenCVToRgb(left_rgb_mat);

    cv::cv2eigen(right_gray_mat,right_gray_pixels);
//    cv::cv2eigen(right_rgb_mat,right_rgb_pixels);
    right_rgb_pixels=convertOpenCVToRgb(right_rgb_mat);

    std::cout<<P1<<std::endl;
    std::cout<<P2<<std::endl;


    Rectified rectified = {
            /*.pixel_left_gray=*/left_gray_pixels,
            /*.pixel_right_gray=*/right_gray_pixels,
            /*.pixel_left_rgb=*/left_rgb_pixels,
            /*.pixel_right_rgb=*/right_rgb_pixels,
                                 left_extrinsics,
                                 right_extrinsics,
                                 R1,
                                 R2,
                                 P1,
                                 P2



    };
    return rectified;
}

