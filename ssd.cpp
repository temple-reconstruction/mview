#include "mview.h"
#include <cassert>

using GrayBlockView = decltype(std::declval<const GrayImage>().block(0, 0, 0, 0)); 
using RgbBlockView = decltype(std::declval<const RgbImage>().block(0, 0, 0, 0)); 

static GrayBlockView gray_block(GrayImageView);
static RgbBlockView rgb_block(RgbImageView);

float ssd_cost_gray(GrayImageView ileft, GrayImageView iright) {
    assert((ileft.height==iright.height)&&("Two imageviews pass into ssd_cost don't have the same height"));
    assert((ileft.width==iright.width)&&("Two imageviews pass into ssd_cost don't have the same width"));
	const GrayBlockView left = gray_block(ileft);
	const GrayBlockView right = gray_block(iright);
	return (left - right).squaredNorm();
}

float ssd_cost_rgb(RgbImageView ileft, RgbImageView iright) {
    assert((ileft.height==iright.height)&&("Two imageviews pass into ssd_cost don't have the same height"));
    assert((ileft.width==iright.width)&&("Two imageviews pass into ssd_cost don't have the same width"));
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

