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

static const std::string data_directory = "data/templeRing/";
static const int nbImages=47;

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
	stereoRectify(intrinsic, {}, intrinsic, {}, imageSize, R, T, R1, R2, P1, P2, Q, 0);//, CV_CALIB_ZERO_DISPARITY);//, 0, -1, imageSize, 0,0);
	cv::Mat map1,map2,map3,map4;
	//cv::Mat out_l=GrayImages[l].clone();
	//cv::Mat out_r=GrayImages[r].clone();
	cv::Mat identity = cv::Mat::eye(3, 3, CV_32F);
    cv::initUndistortRectifyMap(intrinsic,{},R1,P1,imageSize,CV_32FC1,map1,map2);
	cv::initUndistortRectifyMap(intrinsic,{},R2,P2,imageSize,CV_32FC1,map3,map4);
    //cv::remap(GrayImages[l],out_l,map1,map2,cv::INTER_NEAREST,cv::BORDER_CONSTANT); 
    //cv::remap(GrayImages[r],out_r,map3,map4,cv::INTER_NEAREST,cv::BORDER_CONSTANT);

    //std::cout<<map1<<std::endl;
	//std::cout<<intrinsic<<"\n"<<R1<<"\n"<<R2<<"\n"<<P1<<"\n"<<P2<<"\n"<<Q<<"\n"<<R<<"\n"<<T<<std::endl;
	return true;
}

/*static float dist(cv::Mat a, cv::Mat b) {
    return cv::norm(a - b);
}*/

bool write_mesh(std::ostream& outFile, cv::Mat pointcloud, cv::Mat colour, bool use_face=true, bool use_colour=false){
	
	float edgeThreshold = 0.01;
	float minThreshold = 0.2;
	float maxThreshold = 0.2*10;

	int n_row = pointcloud.rows;
	int n_col = pointcloud.cols;

    //number of vertices
	int nVertices = n_row*n_col;

	cv::Mat useful_points(n_row, n_col, CV_32F, 0.0);

	//number of faces
	int nFaces = 0;
	int lt, rt, lb, rb;
	for (int i=0;i<n_row-1;i++){
		for (int j=0;j<n_col-1;j++){
			//check if any point is too far
			if ((pointcloud.at<cv::Vec3f>(i,j)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i,j)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]<maxThreshold)){
				//check if can form a face
				if ((fabs(pointcloud.at<cv::Vec3f>(i,j)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i,j)[2]-pointcloud.at<cv::Vec3f>(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i+1,j)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)){
					if (use_face){
						nFaces++;
					}
					useful_points.at<float>(i,j)=1;
					useful_points.at<float>(i,j+1)=1;
					useful_points.at<float>(i+1,j)=1;
				}
			}
			if ((pointcloud.at<cv::Vec3f>(i+1,j+1)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j+1)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]<maxThreshold)){
				if ((fabs(pointcloud.at<cv::Vec3f>(i+1,j+1)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i+1,j+1)[2]-pointcloud.at<cv::Vec3f>(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i+1,j)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)){
					if (use_face){
						nFaces++;
					}
					useful_points.at<float>(i+1,j+1)=1;
					useful_points.at<float>(i,j+1)=1;
					useful_points.at<float>(i+1,j)=1;
				}
			}
		}
	}

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	//save vertices
    for (int i=0;i<n_row;i++){
		for (int j=0;j<n_col;j++){
			if (useful_points.at<float>(i,j)==0){
            	outFile << "0 0 0 255 255 255 255" << std::endl;
			}
			else{
				if (use_colour){
					outFile << pointcloud.at<cv::Vec3f>(i,j)[0] << " " << pointcloud.at<cv::Vec3f>(i,j)[1] << " " << pointcloud.at<cv::Vec3f>(i,j)[2] << 
			 			colour.at<cv::Vec4d>(i,j)[0] << " " << colour.at<cv::Vec4d>(i,j)[1] << " " << colour.at<cv::Vec4d>(i,j)[2] << " " << colour.at<cv::Vec4d>(i,j)[3] << std::endl;
				}
				else{
					outFile << pointcloud.at<cv::Vec3f>(i,j)[0] << " " << pointcloud.at<cv::Vec3f>(i,j)[1] << " " << pointcloud.at<cv::Vec3f>(i,j)[2] << " 255 255 255 255" << std::endl;
				}
			}
		}
	}

	// save faces
	if(use_face){
    	for (int i=0;i<n_row-1;i++){
			for (int j=0;j<n_col-1;j++){
				lt=i*n_col+j;
				rt=lt+1;
				lb=lt+n_col;
				rb=lb+1;
				if ((pointcloud.at<cv::Vec3f>(i,j)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i,j)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]<maxThreshold)){
					//check if can form a face
					if ((fabs(pointcloud.at<cv::Vec3f>(i,j)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i,j)[2]-pointcloud.at<cv::Vec3f>(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i+1,j)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)){
						outFile << "3 " << lt << " " << rt << " " << lb << std::endl;
					}
				}
				if ((pointcloud.at<cv::Vec3f>(i+1,j+1)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]>minThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j+1)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i,j+1)[2]<maxThreshold)&&(pointcloud.at<cv::Vec3f>(i+1,j)[2]<maxThreshold)){
					if ((fabs(pointcloud.at<cv::Vec3f>(i+1,j+1)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i+1,j+1)[2]-pointcloud.at<cv::Vec3f>(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud.at<cv::Vec3f>(i+1,j)[2]-pointcloud.at<cv::Vec3f>(i,j+1)[2])<edgeThreshold)){
						outFile << "3 " << rt << " " << lb << " " << rb << std::endl;
					}
				}
				/*if((useful_points.at<int>(i,j)*useful_points.at<int>(i,j+1)*useful_points.at<int>(i+1,j))>0){
					outFile << "3 " << lt << " " << rt << " " << lb << std::endl;
				}
				if((useful_points.at<int>(i+1,j+1)*useful_points.at<int>(i,j+1)*useful_points.at<int>(i+1,j))>0){
					outFile << "3 " << rt << " " << lb << " " << rb << std::endl;
				}*/
			}
		}
	}

    return true;
}



int main() {
	std::fstream parameter_file(data_directory + "templeR_par.txt", std::ios_base::in);

	cv::Mat intrinsic(3,3, CV_32F);
	std::vector<cv::Mat> extrinsics;	
	std::vector<std::string> filenames;
	for (int i=0;i<9;i++){
		extrinsics.emplace_back(4,4,CV_32F);
		filenames.push_back("templeR000"+std::to_string(i+1)+".png");
	}
	for (int i=9;i<nbImages;i++){
		extrinsics.emplace_back(4,4,CV_32F);
		filenames.push_back("templeR00"+std::to_string(i)+".png");
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
	cv::Size imageSize = GrayImages[0].size();
	
	int l=0;
	int r=1;
	for (int i=0;i<3;i++){
		std::cout << "Image pairs "+std::to_string(i+1)+"\n";
		std::cout << " Rectifying images\n";
		l=i;
		r=i+1;
		cv::Mat left=GrayImages[l].clone();
		cv::Mat mask_left(left.size(), left.type());
		cv::threshold(left, mask_left, 5, 255, cv::THRESH_BINARY);
		cv::Mat right=GrayImages[r].clone();

		cv::Mat Q(4,4,CV_32F, 100.);
		rectified(l,r, intrinsic, imageSize, extrinsics, GrayImages, left, right, Q);
		cv::imwrite( "./l.png", left);
		cv::imwrite( "./r.png", right);

		std::cout << " Triangulating pointcoulds\n";    
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

		cv::Mat no_background(filtered_disparity.size(),filtered_disparity.type(), 10000.0);

		filtered_disparity.copyTo(no_background, mask_left);
		cv::imwrite( "./d.png", no_background);
		//filtered_disparity/=16.0;

		//cv::Mat QQ=extrinsics[l].clone();
		//QQ.at<float>(0,3)=0.0;
		//QQ.at<float>(1,3)=0.0;
		//QQ.at<float>(2,3)=0.0;

		//cv::Mat QQ=cv::Mat::eye(4,4,CV_32F);
		std::cout<<Q<<std::endl;
		cv::reprojectImageTo3D(no_background, D3, Q, true, -1);//cv::Mat::eye(4,4,CV_32F)

	


		//std::cout << "Merging pointclouds\n";
		//const auto merged = align(pointclouds);
		std::fstream output_file("./output_"+std::to_string(l)+"_"+std::to_string(r)+".off", std::ios_base::out);
		std::cout << " Writing output file\n";
		cv::Mat colour(480,640,CV_16UC4);
		write_mesh(output_file, D3, colour, true, false);
	}
	return 1;
}

