#include "mview.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

static std::string debug_name = "debug.disparity0.png";

Disparity match(const Rectified& rectified) {
	const auto& pixel_left = rectified.pixel_left_gray;
	const auto& pixel_right = rectified.pixel_right_gray;

	cv::Mat left_mat, right_mat;
	cv::eigen2cv(pixel_left, left_mat);
	cv::eigen2cv(pixel_right, right_mat);

	left_mat = left_mat*255.;
	right_mat = right_mat*255.;
	left_mat.assignTo(left_mat, CV_8UC1);
	right_mat.assignTo(right_mat, CV_8UC1);

	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(128, 17);
	Disparity output;
	stereo->compute(left_mat, right_mat, output.disparity);
	cv::imwrite(debug_name, output.disparity);
	debug_name[15]++;

	for(int i = 0; i < output.disparity.rows; i++)
		for(int j = 0; j < output.disparity.cols; j++)
			output.correspondences.push_back({});

	output.disparity.assignTo(output.disparity, CV_32F);
	output.disparity /= 16;

	return output;
}
