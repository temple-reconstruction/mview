#include "mview.h"
#include "xoroshiro.h"

#include <random>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

namespace {
static int debug_count = 0;

class PatchMatch : public Matcher {
public:
	PatchMatch(int max_disp);
	~PatchMatch() override { }
	Disparity match(const Rectified&) override final;
private:
	int max_disp;
	int iterations;
	int random_iterations;
	float min_delta;

	int block_size;
	float gamma;
	float beta;
	float tau_col;
	float tau_grad;

	void initialize(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs);
	void distribute(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs, bool even);
	void random_search(const Rectified& rect, cv::Mat& disparity, cv::Mat& costs);
	float scoring(const Rectified&, int x, int y, float disp);
};

}

PatchMatch::PatchMatch(int max_disp) 
	: max_disp(max_disp),
	  iterations(40),
	  random_iterations(2),
	  min_delta(0.001),
	  block_size(3),
	  gamma(1.),
	  beta(0.),
	  tau_col(.4),
	  tau_grad(1.)
{ }

std::unique_ptr<Matcher> make_patch_matcher() {
	return std::make_unique<PatchMatch>(32);
}

Disparity PatchMatch::match(const Rectified& rectified) {
	cv::Mat disparity(rectified.pixel_left_gray.rows(), rectified.pixel_right_gray.cols(), CV_32F);
	cv::Mat costs(rectified.pixel_left_gray.rows(), rectified.pixel_right_gray.cols(), CV_32F);

	std::cout << disparity.rows << " " << disparity.cols << "\n";

	std::stringstream debug_name;
	std::string debug_left = ((debug_name << "debug.rectified" << debug_count << ".left.png"), debug_name.str()); debug_name.str("");
	std::string debug_right = ((debug_name << "debug.rectified" << debug_count << ".right.png"), debug_name.str()); debug_name.str("");
	std::string debug_disparity = ((debug_name << "debug.disparity" << debug_count << ".png"), debug_name.str()); debug_name.str("");
	std::string debug_cost = ((debug_name << "debug.cost" << debug_count << ".png"), debug_name.str()); debug_name.str("");
	debug_count++;

	initialize(rectified, disparity, costs);
	for(int m = 0; m < iterations; m++) {
		std::cout << "  Patch match iteration\n";
		distribute(rectified, disparity, costs, m%2);
		random_search(rectified, disparity, costs);
	}

	cv::imwrite(debug_disparity, (disparity/max_disp*255.));
	cv::imwrite(debug_cost, costs*255./block_size*block_size);

	std::vector<Correspondence> corrs;
	for(int i = 0; i < disparity.rows; i++)
		for(int j = 0; j < disparity.cols; j++)
			corrs.push_back({});

	return { corrs, disparity };
}

void PatchMatch::initialize(const Rectified& rectified, cv::Mat& disparity, cv::Mat& costs) {
	std::cout << "  Initializing patch match randomly" << std::endl;

	Xoroshiro engine;
	std::uniform_real_distribution<float> distribution(0, max_disp);
	for(int i = 0; i < disparity.rows; i++) {
		for(int j = 0; j < disparity.cols; j++) {
			float disp = distribution(engine);
			disparity.at<float>(i, j) = disp;
		}
	}

	std::cout << "  Calculating initial costs" << std::endl;
	for(int i = 0; i < disparity.rows; i++) {
		for(int j = 0; j < disparity.cols; j++) {
			float disp = disparity.at<float>(i, j);
			costs.at<float>(i, j) = scoring(rectified, i, j, disp);
		}
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
			
			if(chor < cnow && chor < cvert) {
				disparity.at<float>(i, j) = dhor;
				costs.at<float>(i, j) = chor;
			} else if(cvert < cnow) {
				disparity.at<float>(i, j) = dvert;
				costs.at<float>(i, j) = cvert;
			} else { /* No change */ }
		}
	}
}

void PatchMatch::random_search(const Rectified& rectified, cv::Mat& disparity, cv::Mat& costs) {
	Xoroshiro engine;
	std::uniform_real_distribution<float> distribution(-max_disp, max_disp);

	float interval = 1.f;
	float alpha = .5f;

	for(int i = 0; i < disparity.rows; i++) {
		for(int j = 0; j < disparity.cols; j++) {
			float alpha_k = alpha;
			const float base = disparity.at<float>(i, j);
			for(int k = 0; k < random_iterations; k++, alpha_k*=alpha) {
				const float R = distribution(engine); 
				const float delta = interval*alpha_k*R;
				if(std::abs(delta) < min_delta)
					break;

				const float step = base + delta;
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

float PatchMatch::scoring(const Rectified& rectified, int x, int y, float disp) {
	float score = 0.f;
	float weights = 0.f;
	const auto get_in_bounds = [](const GrayImage& mat, int x, int y) {
		x = std::max(std::min(x, (int) mat.rows() - 1), 0);
		y = std::max(std::min(y, (int) mat.cols() - 1), 0);
		return mat(x, y);
	};
	const auto get_interpolated = [&get_in_bounds](const GrayImage& mat, float x, int y) {
		int x_index = static_cast<int>(std::floor(x));
		float x_fract = x - x_index;

		float left_val = get_in_bounds(mat, x_index + 0, y);
		float right_val = get_in_bounds(mat, x_index + 1, y);

		return (1 - x_fract)*left_val + x_fract*right_val;
	};
	const auto& left = rectified.pixel_left_gray;
	const auto& right = rectified.pixel_right_gray;

	float reference_val = get_in_bounds(left, x, y);
	for(int i = x - block_size; i < x + block_size; i++) {
		for(int j = y - block_size; j < y + block_size; j++) {
			float f_val = get_in_bounds(left, i, j);
			float weight = std::exp(-std::abs(f_val - reference_val)/gamma);
			float g_val = get_interpolated(right, i, j - disp);

			float cost = (1 - beta)*std::min(std::abs(f_val - g_val), tau_col)
				+ /*TODO: derivative term*/ 0;
			weights += weight;
			score += weight*cost;
		}
	}

	return score;
}

