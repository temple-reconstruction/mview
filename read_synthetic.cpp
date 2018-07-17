#include "mview.h"
#include "LoaderEXR.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

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

	for(int i = 0; std::getline(parameter, line); i++) {
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
		camera.ground_truth = line + ".exr";
		output.push_back(camera);
	}

	return output;
}

Image read_image(CameraParameter cameraParameter) {
	Image image;
	std::tie(image.gray_pixels, image.rgb_pixels) = ReadImageFromFile(cameraParameter.filename);
	image.extrinsics = cameraParameter.extrinsics;

	image.ground_truth = GrayImage(image.gray_pixels.rows(), image.gray_pixels.cols());
	read_openexr(cameraParameter.ground_truth, image.ground_truth.data(), 
		image.gray_pixels.cols(), image.gray_pixels.rows(), 1);

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
	cv::Mat rgb_mat = cv::imread(filename, cv::IMREAD_COLOR);
	rgb_mat.assignTo(rgb_mat, CV_32F);
	rgb_mat /= 255.;

	RgbImage rgb_target = convertOpenCVToRgb(rgb_mat);
	GrayImage gray_target = rgb_target.unaryExpr([](auto pixel) { return ColourToGray(pixel); }); 

	return {gray_target, rgb_target};
}

float ColourToGray(Eigen::Vector3f rgb) {
	static constexpr float GAMMA = 2.2;

	const Eigen::Vector3f gamma_correct = Eigen::Array3f(rgb).pow(GAMMA);
	return gamma_correct.dot(Eigen::Vector3f { .2126, .7152, .0722 });
}

RgbImage convertOpenCVToRgb(const cv::Mat imageMat){
    cv::Mat bgr[3];
    cv::split(imageMat, bgr);

	int width = imageMat.cols;
	int height = imageMat.rows;
    GrayImage r(height, width), g(height, width), b(height, width);

    cv::cv2eigen(bgr[2], r);
    cv::cv2eigen(bgr[1], g);
    cv::cv2eigen(bgr[0], b);
	std::cout << "converted\n";

    RgbImage rgbImage (g.rows(), g.cols());

    for(int row=0;row<g.rows();row++){
        for(int col=0;col<g.cols();col++){
            rgbImage(row,col)<<r(row,col),g(row,col),b(row,col);
        }
    }

    return rgbImage;
}

