#pragma once
#include "SimpleMesh.h"

#include <memory>
#include <Eigen/Eigen>

struct Cube {
	/// The computed distance to the nearest surface.
	float distance = 1000.f;

	/// The total weight of measurements that went into this metric.
	float weights = 0.f;

	void integrate(float estimated);
};

class SdfIntegrator {
public:
	using coordinate = Eigen::Vector3f;

	SdfIntegrator(int x, int y, int z, coordinate min, coordinate max);

	void set_x_bounds(float amin, float amax) { min[0] = amin; size[0] = amax - amin; }
	void set_y_bounds(float amin, float amax) { min[1] = amin; size[1] = amax - amin; };
	void set_z_bounds(float amin, float amax) { min[2] = amin; size[2] = amax - amin; };

	coordinate coordinate_of(int x, int y, int z) const;
	Eigen::Vector3d pos(int x, int y, int z) const { return coordinate_of(x, y, z).unaryExpr([](float x) { return (double) x; }); }

	int index_of(int x, int y, int z) const;
	Cube& get(int x, int y, int z);
	const Cube& get(int x, int y, int z) const;

	template<typename F,typename=decltype(std::declval<F>()(std::declval<coordinate>(), std::declval<Cube&>()))>
	void visit(F f) {
		for(int x = 0; x < count_x; x++)
			for(int y = 0; y < count_y; y++)
				for(int z = 0; z < count_z; z++)
					f(coordinate_of(x, y, z), get(x, y, z));
	}

	SimpleMesh mesh() const;

private:
	coordinate min;
	coordinate size;

	int count_x;
	int count_y;
	int count_z;

	std::unique_ptr<Cube[]> cubes;
};

