#include "mview.h"
#include <iostream>
#include <fstream>

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif

constexpr static bool USE_FACE = 0;

static float dist(Eigen::Vector3f a, Eigen::Vector3f b) {
    return (a - b).norm();
}

bool write_mesh(std::ostream& outFile, Pointcloud pointcloud, bool use_face)
{
	float edgeThreshold = 0.00001f;

    //number of vertices
	unsigned int nVertices = pointcloud.points.size();

	//number of faces
	unsigned int nFaces = 0;
	if (use_face){
		for (int i=0;i<nVertices-2;i++){
			for (int j=i+1;j<nVertices-1;j++){
            	for (int k=j+1;k<nVertices;k++){
                	if ((pointcloud.points[i](0)!=MINF)&&(pointcloud.points[j](0)!=MINF)&&(pointcloud.points[k](0)!=MINF)){
                    	if ((dist(pointcloud.points[i],pointcloud.points[j])<edgeThreshold)&&(dist(pointcloud.points[i],pointcloud.points[k])<edgeThreshold)&&(dist(pointcloud.points[j],pointcloud.points[k])<edgeThreshold)){
					    	nFaces++;
                    	}
			    	}
            	}
			}
		}
	}

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	//save vertices
    for (int i=0;i<nVertices;i++){
		if (!std::isfinite(pointcloud.points[i](0))){
            outFile << "0 0 0 255 255 255 255" << std::endl;
		}
		else{
			outFile << pointcloud.points[i](0) << " " << pointcloud.points[i](1) << " " << pointcloud.points[i](2) << " " << std::endl;
			 // pointcloud.colours[i](0) << " " << pointcloud.colours[i](1) << " " << pointcloud.colours[i](2) << " " << pointcloud.colours[i](3) << std::endl;
		}
	}

	// save faces
	if(use_face){
    	for (int i=0;i<nVertices-2;i++){
			for (int j=i+1;j<nVertices-1;j++){
            	for (int k=j+1;k<nVertices;k++){
                	if ((pointcloud.points[i](0)!=MINF)&&(pointcloud.points[j](0)!=MINF)&&(pointcloud.points[k](0)!=MINF)){
                    	if ((dist(pointcloud.points[i],pointcloud.points[j])<edgeThreshold)&&(dist(pointcloud.points[i],pointcloud.points[k])<edgeThreshold)&&(dist(pointcloud.points[j],pointcloud.points[k])<edgeThreshold)){
					    	outFile << "3 " << i << " " << j << " " << k << std::endl;
                    	}
			    	}
            	}
			}
		}
	}

    return true;
}
