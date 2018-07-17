#include "mview.h"
#include <opencv2/highgui.hpp>
#include <memory>

static Disparity match(const Rectified& rectified);

namespace /* local types */ {

static std::string debug_name = "debug.disparity0.png";

using Matches = std::vector<Correspondence>;
using PixelCoordinates = Correspondence::PixelCoordinates;

struct MinMatch { 
	int colIndex; 
	float cost;
};

class SsdMatcher: public Matcher {
public:
	SsdMatcher() { }
	Disparity match(const Rectified&) override final;
private:
};

}

std::unique_ptr<Matcher> make_matcher_ssd() {
	return std::make_unique<SsdMatcher>();
}

Disparity SsdMatcher::match(const Rectified& rectified) {
	return ::match(rectified);
}

static constexpr int BLOCK_SIZE = 3;
static MinMatch find_min_gray(GrayImageView pattern, const GrayImage& rhs, int row, int col);
static Correspondence match_to_correspondence(int row, int col, MinMatch match);

Disparity match(const Rectified& rectified) {
	const auto& pixel_left = rectified.pixel_left_gray;
	const auto& pixel_right = rectified.pixel_right_gray;

	assert(pixel_left.rows() == pixel_right.rows());

	Matches matches;
	cv::Mat_<float> distances((int)pixel_left.rows() - 2*BLOCK_SIZE, (int)pixel_left.cols() - 2*BLOCK_SIZE);
	distances.setTo(0.f);

	for(int i = BLOCK_SIZE; i < pixel_left.rows() - BLOCK_SIZE; i++) {
		for(int j = BLOCK_SIZE; j < pixel_left.cols() - BLOCK_SIZE; j++) {
			const GrayImageView pattern { pixel_left, i - BLOCK_SIZE, j - BLOCK_SIZE, 2*BLOCK_SIZE + 1, 2*BLOCK_SIZE + 1 };
			const auto min = find_min_gray(pattern, pixel_right, i, j);

			matches.push_back(match_to_correspondence(i, j, min));
			distances(i - BLOCK_SIZE, j - BLOCK_SIZE) = static_cast<float>(j - min.colIndex);
			if(pixel_left(i, j) < 5./256.) {
				distances(i - BLOCK_SIZE, j - BLOCK_SIZE) = 0;
			}
		}
	}

	cv::Mat disparity = distances.clone();
	disparity *= 16.0f;
	disparity.assignTo(disparity, CV_16U);
	cv::imwrite(debug_name, disparity);
	debug_name[15]++;

	return { matches, distances };
}

MinMatch find_min_gray(GrayImageView pattern, const GrayImage& rhs, int row, int col) {
	MinMatch best_match { 0, std::numeric_limits<float>::max() };

	for(int j = col; j >= std::max(col - 64, BLOCK_SIZE); j--) {
		const GrayImageView compare { rhs, row - BLOCK_SIZE, j - BLOCK_SIZE, 2*BLOCK_SIZE + 1, 2*BLOCK_SIZE + 1 };

		const auto cost = ssd_cost_gray(pattern, compare);
		if(cost < best_match.cost)
			best_match = { j, cost };
	}

	return best_match;
}

Correspondence match_to_correspondence(int row, int col, MinMatch match) {
	const PixelCoordinates coords_left { (float) row, (float) col };
	const PixelCoordinates coords_right { (float) row, (float) match.colIndex };

	return { coords_left, coords_right, match.cost, {} };
}

