#include "mview.h"

namespace /* local types */ {

using Matches = std::vector<Correspondence>;
using PixelCoordinates = Correspondence::PixelCoordinates;

using GrayBlockView = decltype(std::declval<const GrayImage>().block(0, 0, 0, 0)); 
using RgbBlockView = decltype(std::declval<const RgbImage>().block(0, 0, 0, 0)); 

struct MinMatch { 
	int colIndex; 
	float cost;
};

}

static MinMatch find_min_gray(GrayImageView pattern, const GrayImage& rhs, int row);

static Correspondence match_to_correspondence(int row, int col, MinMatch match);
static float ssd_gray(GrayImageView left, GrayImageView right);
static float ssd_rgb(RgbImageView left, RgbImageView right);

static GrayBlockView gray_block(GrayImageView);
static RgbBlockView rgb_block(RgbImageView);

auto match(const Rectified& rectified) -> Matches {
	const auto& pixel_left = rectified.pixel_left_gray;
	const auto& pixel_right = rectified.pixel_right_gray;

	assert(pixel_left.rows() == pixel_right.rows());

	Matches matches;

	for(int i = 0; i < pixel_left.rows(); i++) {
		for(int j = 0; j < pixel_left.cols(); j++) {
			const GrayImageView pattern { pixel_left, i, j, 1, 1 };
			const auto min = find_min_gray(pattern, pixel_right, i);

			matches.push_back(match_to_correspondence(i, j, min));
		}
	}

	return matches;
}

MinMatch find_min_gray(GrayImageView pattern, const GrayImage& rhs, int row) {
	MinMatch best_match { 0, 0.0 };

	for(int j = 0; j < rhs.cols(); j++) {
		const GrayImageView compare { rhs, row, j, 1, 1 };

		const auto cost = ssd_gray(pattern, compare);
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

float ssd_gray(GrayImageView ileft, GrayImageView iright) {
	const GrayBlockView left = gray_block(ileft);
	const GrayBlockView right = gray_block(iright);
	return (left - right).squaredNorm();
}

float ssd_rgb(RgbImageView ileft, RgbImageView iright) {
	const RgbBlockView left = rgb_block(ileft);
	const RgbBlockView right = rgb_block(iright);
	return (left - right).unaryExpr([](auto c){ return c.squaredNorm(); }).sum();
}

GrayBlockView gray_block(GrayImageView view) {
	return view.image.block(view.top_row, view.left_column, view.height, view.width);
}

RgbBlockView rgb_block(RgbImageView view) {
	return view.image.block(view.top_row, view.left_column, view.height, view.width);
}

