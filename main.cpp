#include <iostream>
#include <fstream>

#include "mview.h"

static const std::string data_directory = "data/templeSparseRing/";
static std::vector<Image> read_images(std::vector<CameraParameter> samples);
static std::vector<Rectified> rectified_pairs(const std::vector<Image>& images);
static std::vector<Pointcloud> rectified_to_pointclouds(const std::vector<Rectified>&);

template<typename T, typename F>
static void for_each_pair(T begin, T end, F functor) {
	if(begin == end)
		return;
	for(T first=begin++; begin != end; first++,begin++)
		functor(*first, *begin);
}

int main() {
	std::fstream parameter_file(data_directory + "templeSR_par.txt", std::ios_base::in);
	const auto dataset = read_dataset(parameter_file);
	const auto images = read_images(dataset);
	const auto rectified = rectified_pairs(images);
	const auto pointclouds = rectified_to_pointclouds(rectified);
	const auto merged = align(pointclouds);
	std::fstream output_file("output.ply", std::ios_base::out);
	write_mesh(output_file, merged);
}

std::vector<Image> read_images(std::vector<CameraParameter> samples) {
	std::vector<Image> output (samples.size());
	for(const auto& parameter : samples) {
		output.push_back(read_image(parameter));
	}
	return output;
}

std::vector<Rectified> rectified_pairs(const std::vector<Image>& images) {
	std::vector<Rectified> output (images.size() /* Approximate size */);
	for_each_pair(images.begin(), images.end(), [&](const Image& left, const Image& right) {
		output.push_back(rectify(left, right));
	});
	return output;
}

std::vector<Pointcloud> rectified_to_pointclouds(const std::vector<Rectified>& rectified_pairs) {
	std::vector<Pointcloud> output;
	for(const auto& rectified_pair : rectified_pairs) {
		auto correspondences = match(rectified_pair);
		for(auto& correspondence : correspondences) 
			triangulate(rectified_pair, correspondence);

		Pointcloud pointcloud;
		for(auto& correspondence : correspondences) 
			pointcloud.points.push_back(correspondence.global);
		output.push_back(std::move(pointcloud));
	}
	return output;
}

