#include "sdf.h"

auto SdfIntegrator::coordinate_of(int x, int y, int z) const -> coordinate {
	coordinate dist {
		static_cast<float>(x)/static_cast<float>(count_x - 1)*size[0],
		static_cast<float>(y)/static_cast<float>(count_y - 1)*size[1],
		static_cast<float>(z)/static_cast<float>(count_z - 1)*size[2],
	};

	return min + dist;
}

static float weight_of(float estimated) {
	return 1.;
}

void Cube::integrate(float estimated_distance, float truncation) {
	if(estimated_distance < 0)  // Voxel in front of observation
		return;
	if(estimated_distance > truncation)  // Voxel too far
		return;

	const float previous_sum = distance*weights;
	const float previous_weight = weights;

	const float new_sum = previous_sum + estimated_distance;
	const float new_weights = previous_weight + weight_of(estimated_distance);

	distance = new_sum/new_weights;
	weights = new_weights;
}

int SdfIntegrator::index_of(int x, int y, int z) const {
	// Assumes that z is the fastest index in loops for cache.
	return x*(count_z*count_y) + y*count_z + z;
}

Cube& SdfIntegrator::get(int x, int y, int z) {
	return cubes[index_of(x, y, z)];
}

const Cube& SdfIntegrator::get(int x, int y, int z) const {
	return cubes[index_of(x, y, z)];
}

