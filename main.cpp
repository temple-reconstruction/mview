#include <iostream>
#include <fstream>

#include "mview.h"

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
	std::fstream parameter_file(dataset_file(), std::ios_base::in);
	std::cout << "Reading dataset\n";
	const auto dataset = read_dataset(parameter_file);
	std::cout << "Reading images\n";
	const auto images = read_images(dataset);
	std::cout << "Rectifying images\n";
	const auto rectified = rectified_pairs(images);
	std::cout << "Triangulating pointcoulds\n";
	const auto pointclouds = rectified_to_pointclouds(rectified);
	std::cout << "Merging pointclouds\n";
	const auto merged = align(pointclouds);
	std::fstream output_file("output.off", std::ios_base::out);
	std::cout << "Writing output file\n";
	write_mesh(output_file, merged);
}

std::vector<Image> read_images(std::vector<CameraParameter> samples) {
	std::vector<Image> output;
	output.reserve(samples.size());
	for(auto& parameter : samples) {
		output.push_back(read_image(parameter));
	}
	return output;
}

std::vector<Rectified> rectified_pairs(const std::vector<Image>& images) {
	std::vector<Rectified> output;
 	output.reserve(images.size());
	for_each_pair(images.begin(), images.end(), [&](const Image& left, const Image& right) {
		output.push_back(rectify(left, right));
	});
	return output;
}

std::vector<Pointcloud> rectified_to_pointclouds(const std::vector<Rectified>& rectified_pairs) {
	std::vector<Pointcloud> output;
	int i = 0;
	for(const auto& rectified_pair : rectified_pairs) {
		std::cout << " Finding pixel matches (" << rectified_pair.pixel_left_gray.rows() << "x" << rectified_pair.pixel_left_gray.cols() << ")\n";
		auto disparity = match(rectified_pair);
		std::cout << " Triangulating coordinates\n";

		triangulate(rectified_pair, disparity);

		Pointcloud pointcloud;
		std::cout << " Creating pointcloud\n";
		for(auto& correspondence : disparity.correspondences) {
			pointcloud.points.push_back(correspondence.global);
			pointcloud.colours.push_back(correspondence.colour);
		}

		std::stringstream debug_name;
		debug_name << "output_debug" << i++ << ".off";
		std::fstream debug_out(debug_name.str(), std::ios_base::out);
		std::cout << " Writing debug output\n";
		write_mesh(debug_out, pointcloud);

		output.push_back(std::move(pointcloud));
	}
	return output;
}

