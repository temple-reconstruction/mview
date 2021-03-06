#include "sdf.h"
#include "MarchingCubes.h"
#include <iostream>

SdfIntegrator::SdfIntegrator(int x, int y, int z, coordinate min, coordinate max) :
	min(min), size(max - min),
	count_x(x), count_y(y), count_z(z),
	cubes(new Cube[x*y*z])
{ }

auto SdfIntegrator::coordinate_of(int x, int y, int z) const -> coordinate {
	coordinate dist {
		static_cast<float>(x)/static_cast<float>(count_x - 1)*size[0],
		static_cast<float>(y)/static_cast<float>(count_y - 1)*size[1],
		static_cast<float>(z)/static_cast<float>(count_z - 1)*size[2],
	};

	return min + dist;
}

static float weight_of(float estimated) {
	return 1./(1 + 10.*estimated*estimated);
}

void Cube::integrate(float estimated_distance) {
	const float previous_sum = distance*weights;
	const float previous_weight = weights;

	const float weight = weight_of(estimated_distance);
	const float new_sum = previous_sum + weight*estimated_distance;
	const float new_weights = previous_weight + weight;

	distance = new_sum/new_weights;
	weights = new_weights;
}

void Cube::markFree() {
	freeCtr++;
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

void SdfIntegrator::remove_free() {
	visit([](auto _coord, auto& cube) {
			if(cube.freeCtr > cube.weights)
				cube.distance = 1000.f;
		});
}

SimpleMesh SdfIntegrator::mesh() const {
	SimpleMesh mesh;

	for (int x = 0; x < count_x - 1; x++) {
		std::cerr << "Marching Cubes on slice " << x << " of " << count_x << std::endl;
		for (int y = 0; y < count_y - 1; y++) {
			for (int z = 0; z < count_z - 1; z++) {
				ProcessVolumeCell(*this, x, y, z, 0.f, mesh);
			}
		}
	}

	return mesh;
}
