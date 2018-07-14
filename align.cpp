#include "mview.h"

static Pointcloud align_to_existing(const Pointcloud& reference, const Pointcloud& to_align);
static void merge_pointcloud_into(Pointcloud& target, Pointcloud input);

Pointcloud globalize(const Triangulated& triangulated) {
	Eigen::Matrix4f image_to_global = triangulated.extrinsics.inverse();
	PointImage global = triangulated.points.unaryExpr([&](Eigen::Vector3f point) -> Eigen::Vector3f {
			return image_to_global.block<3, 3>(0, 0) * point 
				+ image_to_global.block<3, 1>(0, 3);
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

constexpr static const float CUTOFF = 0.1f;

void integrate(SdfIntegrator& integrator, const Triangulated& triangulated) {
	const auto visitor = [&](Eigen::Vector3f coords, auto& cube) { 
		const Eigen::Vector3f local_coords = triangulated.extrinsics.block<3, 3>(0, 0) * coords
			+ triangulated.extrinsics.block<3, 1>(0, 3);
		if(local_coords[2] == 0)
			return;

		const Eigen::Vector3f sensor_coords = local_coords/local_coords[2];
		const Eigen::Vector3f image_coords = triangulated.intrinsics * sensor_coords;

		int i = (int) image_coords[1];
		int j = (int) image_coords[0];

		if(i < 0 || i >= triangulated.points.rows())
			return;
		if(j < 0 || j >= triangulated.points.cols())
			return;

		float cube_distance = local_coords.norm();
		float measured = triangulated.points(i, j).norm();

		cube.integrate(cube_distance - measured, CUTOFF);
	};

	integrator.visit(visitor);
}

