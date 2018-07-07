#include <vector>
#include <ostream>
#include <iostream>
#include <istream>
#include <fstream>
#include <string>

//#include "mview.h"
#include "opencv.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

static const std::string data_directory = "data/templeSparseRing/";

static bool read_dataset(std::istream& inputfile, cv::Mat intrinsic, std::vector<cv::Mat> extrinsics, std::vector<std::string> filenames){

	int idx=0;
    std::string line;
    if(!inputfile){
        std::cout<<"cannot open file"<<std::endl;
    }
    std::getline(inputfile,line);
    std::cout<<"imgNum:"<<line<<std::endl;
    while (getline(inputfile,line)){
        //cv::Mat extrinsic(4,4, CV_32F);
        std::istringstream iss(line);
		std::string filename;
		iss>>filename;
        //filenames[idx]=filename;
        std::string s;
        std::string ss[21];
        int i=0;
        while (iss>>s){

            ss[i]=s;
            i++;
        }

        intrinsic.at<float>(0,0)=std::atof(ss[0].c_str());
        intrinsic.at<float>(0,1)=std::atof(ss[1].c_str());
        intrinsic.at<float>(0,2)=std::atof(ss[2].c_str());
        intrinsic.at<float>(1,0)=std::atof(ss[3].c_str());
        intrinsic.at<float>(1,1)=std::atof(ss[4].c_str());
        intrinsic.at<float>(1,2)=std::atof(ss[5].c_str());
        intrinsic.at<float>(2,0)=std::atof(ss[6].c_str());
        intrinsic.at<float>(2,1)=std::atof(ss[7].c_str());
        intrinsic.at<float>(2,2)=std::atof(ss[8].c_str());
        extrinsics[idx].at<float>(0,0)=std::atof(ss[9].c_str());
        extrinsics[idx].at<float>(0,1)=std::atof(ss[10].c_str());
        extrinsics[idx].at<float>(0,2)=std::atof(ss[11].c_str());
        extrinsics[idx].at<float>(1,0)=std::atof(ss[12].c_str());
        extrinsics[idx].at<float>(1,1)=std::atof(ss[13].c_str());
        extrinsics[idx].at<float>(1,2)=std::atof(ss[14].c_str());
        extrinsics[idx].at<float>(2,0)=std::atof(ss[15].c_str());
        extrinsics[idx].at<float>(2,1)=std::atof(ss[16].c_str());
        extrinsics[idx].at<float>(2,2)=std::atof(ss[17].c_str());
        extrinsics[idx].at<float>(0,3)=std::atof(ss[18].c_str());
        extrinsics[idx].at<float>(1,3)=std::atof(ss[19].c_str());
        extrinsics[idx].at<float>(2,3)=std::atof(ss[20].c_str());
        extrinsics[idx].at<float>(3,0)=0.0;
        extrinsics[idx].at<float>(3,1)=0.0;
        extrinsics[idx].at<float>(3,2)=0.0;
        extrinsics[idx].at<float>(3,3)=1.0;
		//extrinsics[idx]=extrinsic;
		idx=idx+1;
    }

    return true;
}

/*static std::vector<cv::Mat> read_images(std::vector<CameraParameter> samples) {
	std::vector<Image> output;
	output.reserve(samples.size());
	for(auto& parameter : samples) {
		parameter.filename = data_directory + parameter.filename;
		output.push_back(read_image(parameter));
	}
	return output;
}

static std::vector<Rectified> rectified_pairs(const std::vector<Image>& images) {
	std::vector<Rectified> output;
 	output.reserve(images.size());
	for_each_pair(images.begin(), images.end(), [&](const Image& left, const Image& right) {
		output.push_back(rectify(left, right));
	});
	return output;
}

static std::vector<Pointcloud> rectified_to_pointclouds(const std::vector<Rectified>& rectified_pairs) {
	std::vector<Pointcloud> output;
	int i = 0;
	for(const auto& rectified_pair : rectified_pairs) {
		std::cout << " Finding pixel matches\n";
		auto correspondences = match(rectified_pair);
		std::cout << " Triangulating coordinates\n";

		triangulate(rectified_pair, correspondences);

		Pointcloud pointcloud;
		std::cout << " Creating pointcloud\n";
		for(auto& correspondence : correspondences) 
			pointcloud.points.push_back(correspondence.global);

		std::stringstream debug_name;
		debug_name << "output_debug" << i++ << ".off";
		std::fstream debug_out(debug_name.str(), std::ios_base::out);
		std::cout << " Writing debug output\n";
		write_mesh(debug_out, pointcloud);

		output.push_back(std::move(pointcloud));
	}
	return output;
}


template<typename T, typename F>
static void for_each_pair(T begin, T end, F functor) {
	if(begin == end)
		return;
	for(T first=begin++; begin != end; first++,begin++)
		functor(*first, *begin);
}*/

static bool rectified(int& l, int& r, cv::Mat intrinsic, cv::Size imageSize, std::vector<cv::Mat> extrinsics, std::vector<cv::Mat> GrayImages, cv::Mat& out_l, cv::Mat& out_r, cv::Mat& Q){
	cv::Mat R_l=extrinsics[l](cv::Range(0,3), cv::Range(0,3));
	cv::Mat R_r=extrinsics[r](cv::Range(0,3), cv::Range(0,3));
	cv::Mat T_l=extrinsics[l].col(3).rowRange(0, 3);
	cv::Mat T_r=extrinsics[r].col(3).rowRange(0, 3);
	cv::Mat R=R_r*(R_l.t());
    cv::Mat T=T_r-R*T_l;
	R.assignTo(R, CV_64F);
	T.assignTo(T, CV_64F);
	cv::Mat R1,R2,P1,P2;
	stereoRectify(intrinsic, {}, intrinsic, {}, imageSize, R, T, R1, R2, P1, P2, Q);//, CV_CALIB_ZERO_DISPARITY);//, 0, -1, imageSize, 0,0);
	cv::Mat map1,map2,map3,map4;
	//cv::Mat out_l=GrayImages[l].clone();
	//cv::Mat out_r=GrayImages[r].clone();
    cv::initUndistortRectifyMap(intrinsic,{},R1,P1,imageSize,CV_32FC1,map1,map2);
	cv::initUndistortRectifyMap(intrinsic,{},R2,P2,imageSize,CV_32FC1,map3,map4);
    //cv::remap(GrayImages[l],out_l,map1,map2,cv::INTER_NEAREST,cv::BORDER_CONSTANT); 
    //cv::remap(GrayImages[r],out_r,map3,map4,cv::INTER_NEAREST,cv::BORDER_CONSTANT);

	//std::cout<<R1<<"\n"<<R2<<"\n"<<P1<<"\n"<<P2<<"\n"<<Q<<"\n"<<R<<"\n"<<T<<std::endl;
	return true;
}

static float dist(cv::Mat a, cv::Mat b) {
    return cv::norm(a - b);
}

bool write_mesh(std::ostream& outFile, cv::Mat pointcloud, bool use_face){
	float edgeThreshold = 0.00001f;

    //number of vertices
	int nVertices = pointcloud.cols*pointcloud.rows;

	//number of faces
	int nFaces = 0;
	if (use_face){ 		
		for (int i=0;i<nVertices-2;i++){
			for (int j=i+1;j<nVertices-1;j++){
            	for (int k=j+1;k<nVertices;k++){
                	//if ((dist(pointcloud.points[i],pointcloud.points[j])<edgeThreshold)&&(dist(pointcloud.points[i],pointcloud.points[k])<edgeThreshold)&&(dist(pointcloud.points[j],pointcloud.points[k])<edgeThreshold)){
					    nFaces++;
			    	//}
            	}
			}
		}
	}

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	//save vertices
    for (int j=0;j<pointcloud.cols;j++){
		for (int i=0;i<pointcloud.rows;i++){
			if (pointcloud.at<cv::Vec3f>(i,j)[2]>1000.0){
            	outFile << "0 0 0 255 255 255 255" << std::endl;
			}
			
			else{
				outFile << pointcloud.at<cv::Vec3f>(i,j)[0] << " " << pointcloud.at<cv::Vec3f>(i,j)[1] << " " << pointcloud.at<cv::Vec3f>(i,j)[2] << " 254 254 254 255" << std::endl;
				// pointcloud.colours[i](0) << " " << pointcloud.colours[i](1) << " " << pointcloud.colours[i](2) << " " << pointcloud.colours[i](3) << std::endl;
			}
		}
	}
	
	// save faces
	if(use_face){
    	for (int i=0;i<nVertices-2;i++){
			for (int j=i+1;j<nVertices-1;j++){
            	for (int k=j+1;k<nVertices;k++){
                	//if ((dist(pointcloud.points[i],pointcloud.points[j])<edgeThreshold)&&(dist(pointcloud.points[i],pointcloud.points[k])<edgeThreshold)&&(dist(pointcloud.points[j],pointcloud.points[k])<edgeThreshold)){
					    outFile << "3 " << i << " " << j << " " << k << std::endl;
			    	//}
            	}
			}
		}
	}

    return true;
}

int main() {
	std::fstream parameter_file(data_directory + "templeSR_par.txt", std::ios_base::in);

	cv::Mat intrinsic(3,3, CV_32F);
	std::vector<cv::Mat> extrinsics;	
	std::vector<std::string> filenames;
	for (int i=0;i<9;i++){
		extrinsics.emplace_back(4,4,CV_32F);
		filenames.push_back("templeSR000"+std::to_string(i+1)+".png");
	}
	for (int i=9;i<16;i++){
		extrinsics.emplace_back(4,4,CV_32F);
		filenames.push_back("templeSR00"+std::to_string(i)+".png");
	}

	std::cout << "Reading dataset\n";
	read_dataset(parameter_file, intrinsic, extrinsics, filenames);
	//std::cout << intrinsic << "\n" << extrinsics[0]<<"\n"<<extrinsics[1] << "\n" << filenames[10] << "\n";

	std::cout << "Reading images\n";
	std::vector<cv::Mat> RGBImages;
	std::vector<cv::Mat> GrayImages;
	for (int i=0;i<16;i++){
		RGBImages.push_back(cv::imread(data_directory+filenames[i] , 1));
		GrayImages.push_back(cv::imread(data_directory+filenames[i] , 0));
	}
	//cv::imwrite( "./a.png", GrayImages[0] );
	//cv::imwrite( "./b.png", RGBImages[0] );

	std::cout << "Rectifying images\n";
	cv::Size imageSize = GrayImages[0].size();

	int l=2;
	int r=3;
	cv::Mat left=GrayImages[l].clone();
	cv::Mat right=GrayImages[r].clone();
	cv::Mat Q(4,4,CV_32F, 100.);
	//left.assignTo(left, CV_8U);
	//right.assignTo(right, CV_8U);
	rectified(l,r, intrinsic, imageSize, extrinsics, GrayImages, left, right, Q);
	//cv::imwrite( "./a.png", left );
	//cv::imwrite( "./b.png", right );

	std::cout << "Triangulating pointcoulds\n";    
	left.assignTo(left, CV_8U);
	right.assignTo(right, CV_8U);
	cv::Ptr<cv::StereoBM> stereoBM_left=cv::StereoBM::create(16,5);
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(stereoBM_left);
	//cv::Ptr<cv::StereoBM> stereoBM_right=cv::StereoBM::create(16,7);
	cv::Ptr<cv::StereoMatcher> stereoBM_right=cv::ximgproc::createRightMatcher(stereoBM_left);
	//cv::Ptr<cv::StereoSGBM> stereoBM_left = cv::StereoSGBM::create(0,16,7, 1,64,1,0, 15, 10,2);
	//cv::Ptr<cv::StereoSGBM> stereoBM_right = cv::StereoSGBM::create(0,16,7, 1,64,1,0, 15, 10,2);
	cv::Mat left_disparity, right_disparity, filtered_disparity;
	stereoBM_left->compute(left, right, left_disparity);
	stereoBM_right->compute(right, left, right_disparity);
	wls_filter->filter(left_disparity,left,filtered_disparity,right_disparity);
	left_disparity.assignTo(left_disparity, CV_32F);
	right_disparity.assignTo(right_disparity, CV_32F);
	filtered_disparity.assignTo(filtered_disparity, CV_32F);
	cv::Mat D3(filtered_disparity.rows,filtered_disparity.cols, CV_32FC3);//=GrayImages[l].clone();
	cv::imwrite( "./a.png", left_disparity);
	cv::imwrite( "./b.png", right_disparity);
	cv::imwrite( "./c.png", filtered_disparity);

	//filtered_disparity/=16.0;

	
	std::cout<<Q<<std::endl;
	cv::reprojectImageTo3D(filtered_disparity, D3, cv::Mat::eye(4,4,CV_32F), true);//cv::Mat::eye(4,4,CV_32F)

	


	//std::cout << "Merging pointclouds\n";
	//const auto merged = align(pointclouds);
	std::fstream output_file("./output.off", std::ios_base::out);
	std::cout << "Writing output file\n";
	write_mesh(output_file, D3, false);

	return 1;
}

