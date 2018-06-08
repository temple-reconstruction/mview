#include "mview.h"

namespace /* local types */ {

using Matches = std::vector<Correspondence>;
using PixelCoordinates = Correspondence::PixelCoordinates;
using BlockView = decltype(std::declval<const Eigen::MatrixXf>().block(0, 0, 0, 0)); 

struct MinMatch { 
	int colIndex; 
	float cost;
};

}

static MinMatch find_min(BlockView pattern, const Eigen::MatrixXf& rhs, int row);
static Correspondence match_to_correspondence(int row, int col, MinMatch match);

auto match(const Rectified& rectified) -> Matches {
	const auto& pixel_left = rectified.pixel_left;
	const auto& pixel_right = rectified.pixel_right;

	assert(pixel_left.rows() == pixel_right.rows());

	Matches matches;

	for(int i = 0; i < pixel_left.rows(); i++) {
		for(int j = 0; j < pixel_left.cols(); j++) {
			const auto min = find_min(pixel_left.block(i, j, 1, 1), pixel_right, i);

			matches.push_back(match_to_correspondence(i, j, min));
		}
	}

	return matches;
}

MinMatch find_min(BlockView pattern, const Eigen::MatrixXf& rhs, int row) {
	MinMatch best_match { 0, 0.0 };

	for(int j = 0; j < rhs.cols(); j++) {
		const BlockView compare = rhs.block(row, j, 1, 1);

		assert(pattern.rows() == compare.rows() && pattern.cols() == compare.cols());

		const auto cost = (pattern - compare).squaredNorm();
		if(cost < best_match.cost)
			best_match = { j, cost };
	}

	return { 0 };
}

Correspondence match_to_correspondence(int row, int col, MinMatch match) {
	const PixelCoordinates coords_left { (float) row, (float) col };
	const PixelCoordinates coords_right { (float) row, (float) match.colIndex };

	return { coords_left, coords_right, match.cost, {} };
}
