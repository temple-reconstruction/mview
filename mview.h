#pragma once
#include <vector>
#include <ostream>
#include <eigen/Eigen.h>

struct CameraParamter {
	std::string filename;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
};

struct Image {
	Eigen::Matrix pixel_values;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
};

struct Rectified {
	Image left;
	Image right;
};

struct Correspondence {
	struct PixelCoordinates {
		float x, y;
	};

	PixelCoordinates left;
	PixelCoordinates right;

	// Estimation of the quality of this correspondence.
	float cost;

	Eigen::Vector3f global;
};

struct ImageView {
	const Image& image;
	const int x, y, w, h;
};

struct Pointcloud {
	std::vector<Eigen::Vector3f> points;
};

float ssd_cost(ImageView left, ImageView right);
auto match(const Rectified&) -> std::vector<Correspondence>;
auto triangulate(const Rectified&, const Correspondence&) -> Eigen::Vector3f;
auto align(std::vector<Pointcloud>) -> Pointcloud;

auto read_dataset(std::istream&) -> std::vector<CameraParameter>;
auto read_dataimage(CameraParameter) -> Image;
/*
 * std::fstream outfile("path");
 * write_mesh(outfile, cloud);
 */
void write_mesh(std::ostream&, Pointcloud);

