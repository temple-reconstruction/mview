#include "mview.h"

static Pointcloud align_to_existing(const Pointcloud& reference, const Pointcloud& to_align);
static void merge_pointcloud_into(Pointcloud& target, Pointcloud input);

Pointcloud globalize(const Triangulated& triangulated) {
	PointImage global = triangulated.points.unaryExpr([&](Eigen::Vector3f point) -> Eigen::Vector3f {
			return triangulated.extrinsics.block<3, 3>(0, 0) * point 
				+ triangulated.extrinsics.block<3, 1>(0, 3);
		});

	Pointcloud pointcloud;
	for(int i = 0; i < global.rows(); i++)
		for(int j = 0; j < global.cols(); j++) {
			pointcloud.points.push_back(global(i, j));
			pointcloud.colours.push_back(triangulated.colour(i, j));
		}

	return pointcloud;
}

auto align(std::vector<Pointcloud> pointclouds) -> Pointcloud {
	Pointcloud output;
	for(const auto& pointcloud : pointclouds) {
		Pointcloud aligned = align_to_existing(output, pointcloud);
		merge_pointcloud_into(output, aligned);
	}
	return output;
}

Pointcloud align_to_existing(const Pointcloud& reference, const Pointcloud& to_align) {
	// FIXME: ICP or another alignment optimization
	return to_align;
}

void merge_pointcloud_into(Pointcloud& target, Pointcloud input) {
	target.points.insert(target.points.end(), input.points.begin(), input.points.end());
}

