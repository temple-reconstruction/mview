#include "mview.h"
#include <opencv2/highgui.hpp>

namespace /* local types */ {

using Matches = std::vector<Correspondence>;
using PixelCoordinates = Correspondence::PixelCoordinates;

struct MinMatch { 
	int colIndex; 
	float cost;
};

}

static constexpr int BLOCK_SIZE = 3;
static MinMatch find_min_gray(GrayImageView pattern, const GrayImage& rhs, int row);
static Correspondence match_to_correspondence(int row, int col, MinMatch match);

auto match(const Rectified& rectified) -> Matches {
	const auto& pixel_left = rectified.pixel_left_gray;
	const auto& pixel_right = rectified.pixel_right_gray;

	assert(pixel_left.rows() == pixel_right.rows());

	Matches matches;
	cv::Mat_<float> distances((int)pixel_left.rows(), (int)pixel_left.cols());
	distances.setTo(0.f);

	for(int i = BLOCK_SIZE; i < pixel_left.rows() - BLOCK_SIZE; i++) {
		for(int j = BLOCK_SIZE; j < pixel_left.cols() - BLOCK_SIZE; j++) {
			if(pixel_left(i, j) < 5./256.)
				continue;
			const GrayImageView pattern { pixel_left, i - BLOCK_SIZE, j - BLOCK_SIZE, 2*BLOCK_SIZE + 1, 2*BLOCK_SIZE + 1 };
			const auto min = find_min_gray(pattern, pixel_right, i);

			matches.push_back(match_to_correspondence(i, j, min));
			distances(i, j) = (min.colIndex - j)/10. + 0.5;
		}
	}

	cv::imshow("distances", distances);
	cv::waitKey(0);

	return matches;
}

MinMatch find_min_gray(GrayImageView pattern, const GrayImage& rhs, int row) {
	MinMatch best_match { 0, std::numeric_limits<float>::max() };

	for(int j = BLOCK_SIZE; j < rhs.cols() - BLOCK_SIZE; j++) {
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

