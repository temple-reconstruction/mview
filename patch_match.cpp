#include "mview.h"
#include <random>

namespace {

class PatchMatch : Matcher {
public:
	PatchMatch(int max_disp);
	Disparity match(const Rectified&) override final;
private:
	int max_disp;
	int iterations;
	int random_iterations;
	float min_delta;

	void initialize(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs);
	void distribute(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs, bool even);
	void random_search(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs);
	float scoring(const Rectified&, int x, int y, float disp);
};

}

PatchMatch::PatchMatch(int max_disp) 
	: max_disp(max_disp),
	  iterations(20),
	  random_iterations(10),
	  min_delta(0.001)
{ }

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

void PatchMatch::distribute(const Rectified& rectified, cv::Mat& disparity, cv::Mat& costs, bool even) {
	int i = even ? 0 : 1;
	int j = even ? 0 : 1;
	int max_i = even ? disparity.rows - 1 : disparity.rows;
	int max_j = even ? disparity.cols - 1 : disparity.cols;
	for(; i < max_i; i++) {
		for(; j < max_j; j++) {
			float cnow = costs.at<float>(i, j);
			int horizontal = even ? 1 : -1;
			int vertical = even ? 1 : -1;

			float dhor = disparity.at<float>(i + horizontal, j);
			float dvert = disparity.at<float>(i, j + vertical);

			float chor = scoring(rectified, i, j, dhor);
			float cvert = scoring(rectified, i, j, dvert);
			
			if(dhor < cnow && dhor < dvert) {
				disparity.at<float>(i, j) = dhor;
				costs.at<float>(i, j) = chor;
			} else if(dvert < cnow) {
				disparity.at<float>(i, j) = dvert;
				costs.at<float>(i, j) = cvert;
			} else { /* No change */ }
		}
	}
}

void PatchMatch::random_search(const Rectified& rectified, cv::Mat& disparity, cv::Mat& costs) {
	std::ranlux48 engine;
	std::uniform_real_distribution<float> distribution(-1, 1);

	float interval = 1.f;
	float alpha = .5f;

	for(int i = 0; i < disparity.rows; i++) {
		for(int j = 0; j < disparity.cols; j++) {
			float alpha_k = alpha;
			for(int k = 0; k < random_iterations; k++, alpha_k*=alpha) {
				const float R = distribution(engine); 
				const float delta = interval*alpha_k*R;
				if(std::abs(delta) < min_delta)
					break;
				const float step = disparity.at<float>(i, j) + delta;
				const float old_score = costs.at<float>(i, j);
				const float new_disp = std::min(std::max(step, 0.f), (float)max_disp);

				const float new_score = scoring(rectified, i, j, new_disp);
				if(new_score >= old_score)
					continue;
				disparity.at<float>(i, j) = new_disp;
				costs.at<float>(i, j) = new_score;
			}
		}
	}
}

float PatchMatch::scoring(const Rectified&, int x, int y, float disp) {
	std::abort();
	return 0.f;
}

