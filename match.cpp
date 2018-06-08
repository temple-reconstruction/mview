#include "mview.h"

using Matches = std::vector<Correspondence>;

auto match(const Rectified& rectified) -> Matches {
	const auto& pixel_left = rectified.pixel_left;
	const auto& pixel_right = rectified.pixel_right;

	assert(pixel_left.rows() == pixel_right.rows());
	Matches matches;

	for(int i = 0; i < pixel_left.rows(); i++) {
		for(int j = 0; j < pixel_left.cols(); j++) {
			// TODO use ssd to find a minimum cost match for each pixel.
		}
	}

	return matches;
}

