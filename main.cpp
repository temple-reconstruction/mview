#include <iostream>
#include <fstream>

#include "mview.h"

static std::vector<Image> read_images(std::vector<CameraParameter> samples);
static std::vector<Rectified> rectified_pairs(const std::vector<Image>& images);
static std::pair<std::vector<Pointcloud>, SdfIntegrator> rectified_integration(const std::vector<Rectified>&);

constexpr static int SPACING = 2;

template<typename T, typename F>
static void for_each_pair(T begin, T end, F functor, int spacing=SPACING) {
	if(begin == end)
		return;
	T first = begin;
	for(int i = 0; i <= spacing && begin != end; i++)
		begin++;
	for(;begin != end; first++,begin++)
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
	const auto result = rectified_integration(rectified);
	const auto& pointclouds = result.first;
	const auto& sdf = result.second;
	std::cout << "Merging pointclouds\n";
	const auto merged = align(pointclouds);
	std::fstream output_file("output.off", std::ios_base::out);
	std::cout << "Writing output file\n";
	// write_mesh(output_file, merged);

	std::cout << "Marching cubes to build mesh\n";
	const auto mesh = sdf.mesh();
	std::cout << "Writing integrated sdf mesh\n";
	std::fstream output_sdf("sdf_mesh.off", std::ios_base::out);
	mesh.WriteMesh(output_sdf);
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

std::pair<std::vector<Pointcloud>, SdfIntegrator> rectified_integration(
	const std::vector<Rectified>& rectified_pairs) 
{
	SdfIntegrator integrator(
		250, 250, 250,
		{1.25, -4.0, 0},
		{3.5, -1.75, 1.75});
	std::vector<Pointcloud> output;
	auto matcher = make_matcher();

	int i = 0;
	for(const auto& rectified_pair : rectified_pairs) {
		std::cout << "Processing rectified pair " << i << " of " << rectified_pairs.size() << "\n";
		std::cout << " Finding pixel matches (" << rectified_pair.pixel_left_gray.rows() << "x" << rectified_pair.pixel_left_gray.cols() << ")\n";
		auto disparity = matcher->match(rectified_pair);
		std::cout << " Triangulating coordinates\n";

		auto triangulated = triangulate(rectified_pair, disparity);
		std::cout << " Integrating into sdf\n";
		integrate(integrator, triangulated);

		std::cout << " Creating pointcloud\n";
		auto pointcloud = globalize(triangulated);

		std::stringstream debug_name;
		debug_name << "output_debug" << i++ << ".off";
		std::fstream debug_out(debug_name.str(), std::ios_base::out);
		std::cout << " Writing debug output\n";
		write_mesh(debug_out, pointcloud);

		output.push_back(std::move(pointcloud));
	}

	return { output, std::move(integrator) };
}

