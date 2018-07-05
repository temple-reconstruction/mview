#pragma once
#include <vector>
#include <ostream>
#include <Eigen/Eigen>

using GrayImage = Eigen::MatrixXf;
using RgbImage = Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic>;

/// Read from the parameter file `<name>_par.txt`
struct CameraParameter {
	std::string filename;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
};

/// A possibly rectified image 
struct Image {
	GrayImage gray_pixels;
	RgbImage rgb_pixels;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
};

/// A pair of images rectified onto a single virtual camera plane with common baselines.
struct Rectified {
    GrayImage pixel_left_gray;
    GrayImage pixel_right_gray;
    
    RgbImage pixel_left_rgb;
    RgbImage pixel_right_rgb;
    
    //    float baseline_distance;
    Eigen::Matrix4f extrinsics_left;
    Eigen::Matrix4f extrinsics_right;
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
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
	Eigen::Vector4d colour;
};

/// A rectangular area of pixel in an image.
struct GrayImageView {
	const GrayImage& image;
	int top_row, left_column, height, width;
};

struct RgbImageView {
	const RgbImage& image;
	int top_row, left_column, height, width;
};

/// Just a collection of 3d (global) points.
struct Pointcloud {
	std::vector<Eigen::Vector3f> points;
	std::vector<Eigen::Vector4d> colours;
};

///@author Yue
float ssd_cost_gray(GrayImageView left, GrayImageView right);
float ssd_cost_rgb(RgbImageView left, RgbImageView right);

///@author Yu
auto rectify(const Image& left, const Image& right) -> Rectified;

///@autor And
auto match(const Rectified&) -> std::vector<Correspondence>;

///@ Sri
/// Fill the missing global coordinate in the correspondence.
void triangulate(const Rectified&, Correspondence&);

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
bool write_mesh(std::ostream& filename, Pointcloud pointcloud, bool use_face=false);

