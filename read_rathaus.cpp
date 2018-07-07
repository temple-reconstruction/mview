#include "mview.h"
#include "FreeImage.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>

static CameraParameter ReadCameraParameterSet(std::string);
static std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string);
static float ColourToGray(Eigen::Vector3f rgb);

auto read_dataset(std::istream& inputfile) -> std::vector<CameraParameter> {
    std::vector<CameraParameter> cameraParameters;
    std::string line;

    while(std::getline(inputfile,line)) {
		cameraParameters.push_back(ReadCameraParameterSet(line));
	}

	return cameraParameters;
}

auto read_image(CameraParameter cameraParameter) -> Image
{
  Image image;
  image.intrinsics = cameraParameter.intrinsics;
  image.extrinsics = cameraParameter.extrinsics;
  std::tie(image.gray_pixels, image.rgb_pixels) = ReadImageFromFile(cameraParameter.filename);

  return image;
}

std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string filename)
{
  std::cout << "Reading " << filename << std::endl;
  std::string linebuffer;
  int w = 3072, h = 2048;

  std::fstream file(filename, std::ios_base::in | std::ios_base::binary);
  file >> linebuffer >> w >> h >> linebuffer;

  std::vector<unsigned char> raw_bytes(w*h*3);
  auto read = file.rdbuf()->sgetn((char*)raw_bytes.data(), raw_bytes.size());
  assert(read == raw_bytes.size());

  RgbImage rgb_target { h, w };
  for(int i = 0; i < w*h; i++)
	for(int s = 0; s < 3; s++)
      rgb_target(i/w, i%w)(s) = static_cast<float>(raw_bytes[3*i+s])/255.f;

  GrayImage gray_target = rgb_target.unaryExpr([](auto color) { return ColourToGray(color); });

  return {gray_target, rgb_target};
}

static CameraParameter ReadCameraParameterSet(std::string line) {
	CameraParameter cameraParameter;
	std::istringstream iss(line);
	iss >> cameraParameter.filename;
	float ignore;

	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			iss >> cameraParameter.intrinsics(i, j);
	
	for(int i = 0; i < 3; i++)
		iss >> ignore;

	Eigen::Matrix3f rotation (3, 3);
	Eigen::Vector3f translation (3, 1);
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			iss >> rotation(i, j);
	for(int i = 0; i < 3; i++)
		iss >> translation(i, 0);

	cameraParameter.extrinsics.block<3,3>(0, 0) = rotation.transpose();
	cameraParameter.extrinsics.block<3,1>(0, 3) = -rotation.transpose()*translation;
	cameraParameter.extrinsics(3, 3) = 1.0;
	return cameraParameter;
}

static float ColourToGray(Eigen::Vector3f rgb) {
  static constexpr float GAMMA = 2.2;

  const Eigen::Vector3f gamma_correct = Eigen::Array3f(rgb).pow(GAMMA);
  return gamma_correct.dot(Eigen::Vector3f { .2126, .7152, .0722 });
}

