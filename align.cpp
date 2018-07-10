#include "mview.h"

static Pointcloud align_to_existing(const Pointcloud& reference, const Pointcloud& to_align);
static void merge_pointcloud_into(Pointcloud& target, Pointcloud input);

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
