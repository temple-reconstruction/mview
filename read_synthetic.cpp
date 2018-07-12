#include "mview.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "FreeImage.h"

static std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string);
static float ColourToGray(Eigen::Vector3f rgb);

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
	image.intrinsics.setIdentity();
	image.intrinsics(0, 0) = image_width*4.1/4.54;
	image.intrinsics(1, 1) = image_height*4.1/3.42;
	image.intrinsics(0, 2) = image_width/2.;
	image.intrinsics(1, 2) = image_height/2.;

	return image;
}

std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string filename) {
	std::cout << filename << std::endl;
	FreeImage_Initialise();

	//image format
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	//pointer to the image, once loaded
	FIBITMAP *dib = nullptr;

	//check the file signature and deduce its format
	fif = FreeImage_GetFileType(filename.c_str(), 0);
	if (fif == FIF_UNKNOWN)
		fif = FreeImage_GetFIFFromFilename(filename.c_str());
	if (fif == FIF_UNKNOWN)
		throw std::runtime_error("Data image format can not be determined");

	//check that the plugin has reading capabilities and load the file
	if (FreeImage_FIFSupportsReading(fif))
		dib = FreeImage_Load(fif, filename.c_str());
	if (!dib)
		throw std::runtime_error("Image could not be read");

	// Convert to RGBA float images
	FIBITMAP *hOldImage = dib;
	dib = FreeImage_ConvertToRGBAF(hOldImage); // ==> 4 channels
	FreeImage_Unload(hOldImage);

	//get the image width and height
	int w = FreeImage_GetWidth(dib);
	int h = FreeImage_GetHeight(dib);

	//retrieve the image data
	auto bits = FreeImage_GetBits(dib);

	//if this somehow one of these failed (they shouldn't), return failure
	if ((bits == 0) || (w == 0) || (h == 0))
		throw std::runtime_error("Could not convert to rgb");

	// Eigen stores column major by default, we prefer pixel samples in row major.
	using PixelSampleMatrix = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

	// Map the pixel values skipping the alpha value by a larger stride.
	using PixelSamples = Eigen::Map<PixelSampleMatrix, 0, Eigen::Stride<4, 1>>;
	PixelSamples data_vector { reinterpret_cast<float*>(bits), w*h, 3 };

	RgbImage rgb_target { h, w };
	for(int i = 0; i < w*h; i++)
		rgb_target(i/w, i%w) = data_vector.row(i);

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

