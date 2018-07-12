#include "mview.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

static int debug_count = 0;

Disparity match(const Rectified& rectified) {
	// If the images are vertically aligned, we transpose for matching
	bool transpose = false; // rectified.P2.at<float>(0, 3) == 0;
	std::stringstream debug_name;
	std::string debug_left = ((debug_name << "debug.rectified" << debug_count << ".left.png"), debug_name.str()); debug_name.str("");
	std::string debug_right = ((debug_name << "debug.rectified" << debug_count << ".right.png"), debug_name.str()); debug_name.str("");
	std::string debug_disparity = ((debug_name << "debug.disparity" << debug_count << ".png"), debug_name.str()); debug_name.str("");
	debug_count++;

	GrayImage pixel_left = transpose ? rectified.pixel_left_gray.transpose() : rectified.pixel_left_gray;
	GrayImage pixel_right = transpose ? rectified.pixel_right_gray.transpose() : rectified.pixel_right_gray;

	cv::Mat left_mat, right_mat;
	cv::eigen2cv(pixel_left, left_mat);
	cv::eigen2cv(pixel_right, right_mat);

	left_mat = left_mat*255.;
	right_mat = right_mat*255.;
	left_mat.assignTo(left_mat, CV_8UC1);
	right_mat.assignTo(right_mat, CV_8UC1);

	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(64, 5);
	stereo->setMinDisparity(-32);

	Disparity output;
	stereo->compute(left_mat, right_mat, output.disparity);
	if(transpose)
		output.disparity = output.disparity.t();

	cv::imwrite(debug_left, left_mat);
	cv::imwrite(debug_right, right_mat);
	cv::imwrite(debug_disparity, output.disparity);

	for(int i = 0; i < output.disparity.rows; i++)
		for(int j = 0; j < output.disparity.cols; j++)
			output.correspondences.push_back({});

	output.disparity.assignTo(output.disparity, CV_32F);
	output.disparity /= 16.;
	output.disparity.setTo(-1000., output.disparity < stereo->getMinDisparity());

	return output;
}
