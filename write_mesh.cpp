#include "mview.h"
#include <iostream>
#include <fstream>

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif

constexpr static bool USE_FACE = 0;

/*static float dist(Eigen::Vector3f a, Eigen::Vector3f b) {
    return (a - b).norm();
}*/

bool write_mesh(std::ostream& outFile, Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, Eigen::Dynamic> pointcloud,  Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, Eigen::Dynamic> colour, bool use_face=true, bool use_colour=false){
	
	//please change to a good number...	
	float edgeThreshold = 2.0;
	float minThreshold = -0.1*1520;
	float maxThreshold = 0.1*1520;

	int n_row = pointcloud.rows();
	int n_col = pointcloud.cols();

    //number of vertices
	int nVertices = n_row*n_col;

	Eigen::MatrixXf useful_points(n_row, n_col, 0.0);

	//number of faces
	int nFaces = 0;
	int lt, rt, lb, rb;
	for (int i=0;i<n_row-1;i++){
		for (int j=0;j<n_col-1;j++){
			//check if any point is too far
			if ((pointcloud(i,j)[2]>minThreshold)&&(pointcloud(i,j+1)[2]>minThreshold)&&(pointcloud(i+1,j)[2]>minThreshold)&&(pointcloud(i,j)[2]<maxThreshold)&&(pointcloud(i,j+1)[2]<maxThreshold)&&(pointcloud(i+1,j)[2]<maxThreshold)){
				//check if can form a face
				if ((fabs(pointcloud(i,j)[2]-pointcloud(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud(i,j)[2]-pointcloud(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud(i+1,j)[2]-pointcloud(i,j+1)[2])<edgeThreshold)){
					if (use_face){
						nFaces++;
					}
					useful_points(i,j)=1;
					useful_points(i,j+1)=1;
					useful_points(i+1,j)=1;
				}
			}
			if ((pointcloud(i+1,j+1)[2]>minThreshold)&&(pointcloud(i,j+1)[2]>minThreshold)&&(pointcloud(i+1,j)[2]>minThreshold)&&(pointcloud(i+1,j+1)[2]<maxThreshold)&&(pointcloud(i,j+1)[2]<maxThreshold)&&(pointcloud(i+1,j)[2]<maxThreshold)){
				if ((fabs(pointcloud(i+1,j+1)[2]-pointcloud(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud(i+1,j+1)[2]-pointcloud(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud(i+1,j)[2]-pointcloud(i,j+1)[2])<edgeThreshold)){
					if (use_face){
						nFaces++;
					}
					useful_points(i+1,j+1)=1;
					useful_points(i,j+1)=1;
					useful_points(i+1,j)=1;
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
			if (useful_points(i,j)==0.0){
            	outFile << "0 0 0 255 255 255 255" << std::endl;
			}
			else{
				if (use_colour){
					outFile << pointcloud(i,j)[0] << " " << pointcloud(i,j)[1] << " " << pointcloud(i,j)[2] << 
			 			colour(i,j)[0] << " " << colour(i,j)[1] << " " << colour(i,j)[2] << " " << colour(i,j)[3] << std::endl;
				}
				else{
					outFile << pointcloud(i,j)[0] << " " << pointcloud(i,j)[1] << " " << pointcloud(i,j)[2] << " 255 255 255 255" << std::endl;
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
				if ((pointcloud(i,j)[2]>minThreshold)&&(pointcloud(i,j+1)[2]>minThreshold)&&(pointcloud(i+1,j)[2]>minThreshold)&&(pointcloud(i,j)[2]<maxThreshold)&&(pointcloud(i,j+1)[2]<maxThreshold)&&(pointcloud(i+1,j)[2]<maxThreshold)){
					if ((fabs(pointcloud(i,j)[2]-pointcloud(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud(i,j)[2]-pointcloud(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud(i+1,j)[2]-pointcloud(i,j+1)[2])<edgeThreshold)){
						outFile << "3 " << lt << " " << rt << " " << lb << std::endl;
					}
				}
				if ((pointcloud(i+1,j+1)[2]>minThreshold)&&(pointcloud(i,j+1)[2]>minThreshold)&&(pointcloud(i+1,j)[2]>minThreshold)&&(pointcloud(i+1,j+1)[2]<maxThreshold)&&(pointcloud(i,j+1)[2]<maxThreshold)&&(pointcloud(i+1,j)[2]<maxThreshold)){
					if ((fabs(pointcloud(i+1,j+1)[2]-pointcloud(i,j+1)[2])<edgeThreshold)&&(fabs(pointcloud(i+1,j+1)[2]-pointcloud(i+1,j)[2])<edgeThreshold)&&(fabs(pointcloud(i+1,j)[2]-pointcloud(i,j+1)[2])<edgeThreshold)){
						outFile << "3 " << rt << " " << lb << " " << rb << std::endl;
					}
				}
			}
		}
	}

    return true;
}
