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
	// FIXME: do we need ICP or another alignment optimization
	return to_align;
}

void merge_pointcloud_into(Pointcloud& target, Pointcloud input) {
	target.points.insert(target.points.end(), input.points.begin(), input.points.end());
	target.colours.insert(target.colours.end(), input.colours.begin(), input.colours.end());
}

constexpr static const float OUTER_CUTOFF = 0.4f;
constexpr static const float INNER_CUTOFF = -0.08f;

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

		if(i < 0 || i >= triangulated.points.rows() - 1)
			return;
		if(j < 0 || j >= triangulated.points.cols() - 1)
			return;

		// i and j have been rounded down, because of >= 0
		const float i_fract = image_coords[1] - i;
		const float j_fract = image_coords[0] - j;

		const float cube_distance = local_coords.norm();

		const float measured_b = triangulated.points(i, j).norm();
		const float measured_t = triangulated.points(i + 1, j).norm();
		const float measured_r = triangulated.points(i, j + 1).norm();
		const float measured_tr = triangulated.points(i + 1, j + 1).norm();

		const float measured = (1 - i_fract)*(1 - j_fract)*measured_b
			+ i_fract*(1 - j_fract)*measured_t
			+ (1 - i_fract)*j_fract*measured_r
			+ i_fract*j_fract*measured_tr;

		const float signed_distance = cube_distance - measured;

		if(signed_distance < INNER_CUTOFF)
			return;
		if(signed_distance > OUTER_CUTOFF)
			return cube.markFree();

		cube.integrate(signed_distance);
	};

	integrator.visit(visitor);
}

