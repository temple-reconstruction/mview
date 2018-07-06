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
	std::fstream output_file("output.ply", std::ios_base::out);
	std::cout << "Writing output file\n";
	write_mesh(output_file, merged);
}

std::vector<Image> read_images(std::vector<CameraParameter> samples) {
	std::vector<Image> output;
	output.reserve(samples.size());
	for(auto& parameter : samples) {
		parameter.filename = data_directory + parameter.filename;
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
		std::cout << " Finding pixel matches\n";
		auto correspondences = match(rectified_pair);
		std::cout << " Triangulating coordinates\n";
		// for(auto& correspondence : correspondences) 
		triangulate(rectified_pair, correspondences);

		Pointcloud pointcloud;
		std::cout << " Creating pointcloud\n";
		for(auto& correspondence : correspondences) 
			pointcloud.points.push_back(correspondence.global);

		std::stringstream debug_name;
		debug_name << "output_debug" << i++ << ".ply";
		std::fstream debug_out(debug_name.str(), std::ios_base::out);
		std::cout << " Writing debug output\n";
		for(auto& correspondence : correspondences) 
		write_mesh(debug_out, pointcloud);

		output.push_back(std::move(pointcloud));
	}
	return output;
}

// void triangulate(const Rectified&, Correspondence&) { }

