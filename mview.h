#pragma once
#include <memory>
#include <ostream>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "sdf.h"
#include "MarchingCubes.h"

#define MVIEW_NDEBUG 0

using GrayImage = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RgbImage = Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using PointImage = Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic>;
using ColourImage = Eigen::Matrix<Eigen::Vector4f, Eigen::Dynamic, Eigen::Dynamic>;

/// Read from the parameter file `<name>_par.txt`
struct CameraParameter {
	std::string filename;
	std::string ground_truth;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
};

/// A possibly rectified image 
struct Image {
	GrayImage gray_pixels;
	RgbImage rgb_pixels;
	GrayImage ground_truth;
	Eigen::Matrix3f intrinsics;
	Eigen::Matrix4f extrinsics;
};

/// A pair of images rectified onto a single virtual camera plane with common baselines.
struct Rectified {
    GrayImage pixel_left_gray;
    GrayImage pixel_right_gray;
	GrayImage left_ground_truth;
    
    RgbImage pixel_left_rgb;
    RgbImage pixel_right_rgb;
	GrayImage right_ground_truth;
    
    //    float baseline_distance;
    Eigen::Matrix4f extrinsics_left;
    Eigen::Matrix4f extrinsics_right;

    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;

	cv::Mat Q;
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
	Eigen::Vector4f colour;
};

struct Disparity {
	std::vector<Correspondence> correspondences;
	cv::Mat disparity;
};

/// Points reconstructed from disparity.
struct Triangulated {
	PointImage points;
	ColourImage colour;

	// Worlds to camera extrinsics.
	Eigen::Matrix4f extrinsics;
	Eigen::Matrix3f intrinsics;
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
	std::vector<Eigen::Vector4f> colours;
};

///@author Yue
float ssd_cost_gray(GrayImageView left, GrayImageView right);
float ssd_cost_rgb(RgbImageView left, RgbImageView right);

///@author Yu
auto rectify(const Image& left, const Image& right) -> Rectified;

class Matcher {
public:
	virtual ~Matcher() { };
	virtual Disparity match(const Rectified&) = 0;
};

namespace std {
	template<>
	struct default_delete<Matcher> {
		default_delete() noexcept = default;
		template<typename T,typename=
			typename std::enable_if<std::is_convertible<T*,Matcher*>::value>::type>
		default_delete(const default_delete<T>&) noexcept
			{ }
		void operator()(Matcher*) const;
	};
}

///@autor And
std::unique_ptr<Matcher> make_matcher_ssd();
std::unique_ptr<Matcher> make_matcher();
std::unique_ptr<Matcher> make_patch_matcher();

///@ Sri
/// Fill the missing global coordinate in the correspondence.
Triangulated triangulate(const Rectified&, const Disparity&);

Pointcloud globalize(const Triangulated&);

void integrate(SdfIntegrator&, const Triangulated&);

///@ And, Yue
auto align(std::vector<Pointcloud>) -> Pointcloud;

std::string dataset_file();

///@ Yu
auto read_dataset(std::istream&) -> std::vector<CameraParameter>;

///@ Sri
auto read_image(CameraParameter) -> Image;

/**
 * std::fstream outfile("path");
 * write_mesh(outfile, cloud);
 */
///@ Yue
bool write_mesh(std::ostream& filename, const Pointcloud& pointcloud, bool use_face=false);
bool write_view_mesh(std::ostream& outFile, const PointImage& pointcloud, const ColourImage& colour, bool use_face=true, bool use_colour=false);

///@ And
void integrate(SdfIntegrator&, const Pointcloud&);

