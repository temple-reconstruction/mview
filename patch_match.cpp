#include "mview.h"
#include <random>

namespace {

class PatchMatch : Matcher {
public:
	using score_fn = float(*)(const Rectified&, int x, int y, float disp);

	PatchMatch(int max_disp);
	Disparity match(const Rectified&) override final;

	void set_score_fn(score_fn);
private:
	score_fn scoring;
	int max_disp;
	int iterations;

	void initialize(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs);
	void distribute(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs, bool even);
	void random_search(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs);
};

}

Disparity PatchMatch::match(const Rectified& rectified) {
	cv::Mat disparity(rectified.pixel_left_gray.rows(), rectified.pixel_right_gray.cols(), CV_32F);
	cv::Mat costs(rectified.pixel_left_gray.rows(), rectified.pixel_right_gray.cols(), CV_32F);

	initialize(rectified, disparity, costs);
	for(int m = 0; m < iterations; m++) {
		distribute(rectified, disparity, costs, m%2);
		random_search(rectified, disparity, costs);
	}

	std::vector<Correspondence> corrs;
	for(int i = 0; i < disparity.rows; i++)
		for(int j = 0; j < disparity.cols; j++)
			corrs.push_back({});

	return { corrs, disparity };
}

void PatchMatch::initialize(const Rectified& rectified, cv::Mat& disparity, cv::Mat& costs) {
	std::ranlux48 engine;

	std::uniform_int_distribution<int> dist(0, max_disp);
	for(int i = 0; i < disparity.rows; i++)
		for(int j = 0; j < disparity.cols; j++) {
			float disp = dist(engine);
			disparity.at<float>(i, j) = disp;
			costs.at<float>(i, j) = scoring(rectified, i, j, disp);
		}
}

