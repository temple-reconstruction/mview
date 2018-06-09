#pragma once
#include <vector>
#include <ostream>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>

using RgbImage = Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic>;

/// Read from the parameter file `<name>_par.txt`
struct CameraParameter {
	std::string filename;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
};

/// A possibly rectified image 
struct Image {
	RgbImage RGB_pixel_values;
	Eigen::MatrixXf Gray_pixel_values;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
	int width;
	int height;
};

/// A pair of images rectified onto a single virtual camera plane with common baselines.
struct Rectified {
	Eigen::MatrixXf pixel_left;
	Eigen::MatrixXf pixel_right;

	float baseline_distance;
	Eigen::Matrix4f extrinsics;
};

/// A pair of pixels allegedly corresponding to the same object vertex.
struct Correspondence {
	struct PixelCoordinates {
		float x, y;
	};

	/// Coordinates relative to the left rectified image.
	PixelCoordinates left;

	/// Coordinates relative to the right rectified image.
	PixelCoordinates right;

	// Estimation of the quality of this correspondence (0 = best, ∞ = worst).
	float cost;

	// Reconstructed global coordinates, filled later through triangulate.
	Eigen::Vector3f global;
};

/// A rectangular area of pixel in an image.
struct ImageView {
	const Image& image;
	int top_row, left_column, height, width;
};

/// Just a collection of 3d (global) points.
struct Pointcloud {
	std::vector<Eigen::Vector3f> points;
};

///@author Yue
float ssd_cost_gray(ImageView left, ImageView right);

float ssd_cost_RGB(ImageView left, ImageView right);

///@author Yu
auto rectify(const Image& left, const Image& right) -> Rectified;

///@autor And
auto match(const Rectified&) -> std::vector<Correspondence>;

///@ Sri
/// Fill the missing global coordinate in the correspondence.
auto triangulate(const Rectified&, Correspondence&);

///@ And, Yue
auto align(std::vector<Pointcloud>) -> Pointcloud;

///@ Yu
auto read_dataset(std::istream&) -> std::vector<CameraParameter>;

///@ Sri
auto read_image(CameraParameter) -> Image;

/**
 * std::fstream outfile("path");
 * write_mesh(outfile, cloud);
 */
///@ Yue
bool write_mesh(const std::string& filename, Pointcloud pointcloud, Pointcloud colour);

