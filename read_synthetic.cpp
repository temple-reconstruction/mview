#include "mview.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

static std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string);
static float ColourToGray(Eigen::Vector3f rgb);
static RgbImage convertOpenCVToRgb(cv::Mat rgbMat);

static const std::string data_directory = "data/output640x480/";

std::string dataset_file() {
	return data_directory + "parameter.txt";
}

std::vector<CameraParameter> read_dataset(std::istream& parameter) {
	std::vector<CameraParameter> output;
	std::string line;

	for(int i = 0; i < 30 && std::getline(parameter, line); i++) {
		CameraParameter camera;
		std::ifstream matrix_file(line+".dat");

		camera.extrinsics.setIdentity();
		for(int j = 0; j < 4; j++)
			for(int i = 0; i < 3; i++)
				matrix_file >> camera.extrinsics(i, j);
		for(int j = 0; j < 3; j++)
			camera.extrinsics.col(j).normalize();
		// camera.extrinsics = camera.extrinsics.inverse().eval();
		camera.filename = line + ".jpg";
		output.push_back(camera);
	}

	return output;
}

Image read_image(CameraParameter cameraParameter) {
	Image image;
	std::tie(image.gray_pixels, image.rgb_pixels) = ReadImageFromFile(cameraParameter.filename);
	image.extrinsics = cameraParameter.extrinsics;

	/*
	INTRINSIC:

	float fx = IMAGE_WIDTH*4.1/4.54, fy = IMAGE_HEIGHT*4.1/3.42, 
	cx = IMAGE_WIDTH/2.0, cy = IMAGE_HEIGHT/2.0;
	*/
	const float image_width = image.gray_pixels.cols();
	const float image_height = image.gray_pixels.rows();
	std::cout << image_width << "x" << image_height << "\n";
	image.intrinsics.setIdentity();
	image.intrinsics(0, 0) = image_width*4.1/4.54;
	image.intrinsics(1, 1) = image_height*4.1/3.42;
	image.intrinsics(0, 2) = image_width/2.;
	image.intrinsics(1, 2) = image_height/2.;
	std::cout << image.intrinsics << "\n";

	return image;
}

std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string filename) {
	std::cout << filename << std::endl;
	cv::Mat rgb_mat = cv::imread(filename);

	// Eigen stores column major by default, we prefer pixel samples in row major.
	using PixelSampleMatrix = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

	// Map the pixel values skipping the alpha value by a larger stride.
	using PixelSamples = Eigen::Map<PixelSampleMatrix, 0, Eigen::Stride<4, 1>>;
	PixelSamples data_vector { reinterpret_cast<float*>(bits), w*h, 3 };

	RgbImage rgb_target { h, w };
	for(int i = 0; i < w*h; i++)
		rgb_target(h - 1 - i/w, i%w) = data_vector.row(i);

	GrayImage gray_target = rgb_target.unaryExpr([](auto pixel) { return ColourToGray(pixel); }); 

	//Free FreeImage's copy of the data
	FreeImage_Unload(dib);

	return {gray_target, rgb_target};
}

float ColourToGray(Eigen::Vector3f rgb) {
	static constexpr float GAMMA = 2.2;

	const Eigen::Vector3f gamma_correct = Eigen::Array3f(rgb).pow(GAMMA);
	return gamma_correct.dot(Eigen::Vector3f { .2126, .7152, .0722 });
}

RgbImage convertOpenCVToRgb(const cv::Mat rgbMat){
    std::array<cv::Mat, 3> rgb;
    cv::split(rgbMat,rgb.data());
    GrayImage r,g,b;
    cv::cv2eigen(rgb[0],r);
    cv::cv2eigen(rgb[1],g);
    cv::cv2eigen(rgb[2],b);

    RgbImage rgbImage (g.rows(), g.cols());
// Eigen::Matrix<Eigen::Vector3f,480,640> rgbImage;
//    rgbImage.setZero();
    for(int row=0;row<g.rows();row++){
        for(int col=0;col<g.cols();col++){
            rgbImage(row,col)<<r(row,col),g(row,col),b(row,col);
        }
    }
//    std::cout<<rgbImage.rows()<<std::endl;
    return rgbImage;
}

