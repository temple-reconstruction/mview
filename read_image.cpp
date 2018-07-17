#include "mview.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

static std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string);
static float ColourToGray(Eigen::Vector3f rgb);
static RgbImage convertOpenCVToRgb(cv::Mat rgbMat);

auto read_image(CameraParameter cameraParameter) -> Image
{
  Image image;
  image.intrinsics = cameraParameter.intrinsics;
  image.extrinsics = cameraParameter.extrinsics;
  std::tie(image.gray_pixels, image.rgb_pixels) = ReadImageFromFile(cameraParameter.filename);

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

RgbImage convertOpenCVToRgb(const cv::Mat rgbMat){
    cv::Mat rgb[3];
    cv::split(rgbMat, rgb);

	int width = rgbMat.cols;
	int height = rgbMat.rows;
    GrayImage r(height, width), g(height, width), b(height, width);

    cv::cv2eigen(rgb[0], r);
    cv::cv2eigen(rgb[1], g);
    cv::cv2eigen(rgb[2], b);
	std::cout << "converted\n";

    RgbImage rgbImage (g.rows(), g.cols());

    for(int row=0;row<g.rows();row++){
        for(int col=0;col<g.cols();col++){
            rgbImage(row,col)<<r(row,col),g(row,col),b(row,col);
        }
    }

    return rgbImage;
}

