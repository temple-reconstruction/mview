#include "mview.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

static int debug_count = 0;

static Disparity match(const Rectified& rectified);

namespace {

class CvMatcher : public Matcher {
public:
	CvMatcher();
	~CvMatcher() override { }
	Disparity match(const Rectified&) override final;
private:
	cv::Ptr<cv::StereoSGBM> inner;
};

}

CvMatcher::CvMatcher() {
	inner = cv::StereoSGBM::create(0, 32, 7,
			8, 32,
			0, 0,
			15,
		   	50, 1);
}

Disparity CvMatcher::match(const Rectified& rectified) {
	return ::match(rectified);
}

std::unique_ptr<Matcher> make_matcher() {
	std::unique_ptr<Matcher> value = std::make_unique<CvMatcher>();
	return std::move(value);
}

Disparity match(const Rectified& rectified) {
	// If the images are vertically aligned, we transpose for matching
	bool vertical = false; // rectified.P2.at<float>(1, 3) != 0;
	std::stringstream debug_name;
	std::string debug_disparity = ((debug_name << "debug.disparity" << debug_count << ".png"), debug_name.str()); debug_name.str("");
	debug_count++;

	GrayImage pixel_left = rectified.pixel_left_gray;
	GrayImage pixel_right = rectified.pixel_right_gray;

	if(vertical) {
		pixel_left = pixel_left.transpose().eval();
		pixel_right = pixel_right.transpose().eval();
	}

	cv::Mat left_mat, right_mat, depth_mat;
	cv::eigen2cv(pixel_left, left_mat);
	cv::eigen2cv(pixel_right, right_mat);
	cv::eigen2cv(rectified.left_ground_truth, depth_mat);

	left_mat = left_mat*255.;
	right_mat = right_mat*255.;
	left_mat.assignTo(left_mat, CV_8UC1);
	right_mat.assignTo(right_mat, CV_8UC1);

	cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(0, 32, 7,
			8, 32,
			0, 0,
			15,
		   	50, 1);
	// stereo->setMinDisparity(0);

	Disparity output;
	stereo->compute(left_mat, right_mat, output.disparity);
	if(vertical)
		output.disparity = output.disparity.t();

	for(int i = 0; i < output.disparity.rows; i++)
		for(int j = 0; j < output.disparity.cols; j++)
			output.correspondences.push_back({});

	output.disparity.assignTo(output.disparity, CV_32F);
	output.disparity /= 16.;

	cv::imwrite(debug_disparity, (output.disparity - stereo->getMinDisparity())/stereo->getNumDisparities()*255.);
	output.disparity.setTo(-1000., output.disparity < stereo->getMinDisparity());

	return output;
}
