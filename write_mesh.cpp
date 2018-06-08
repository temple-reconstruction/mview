#include "mview.h"
#include <iostream>
#include <fstream>

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif

float dist(Eigen::VectorXf a, Eigen::VectorXf b){
    return sqrt(pow((a(0)-b(0)),2)+pow((a(1)-b(1)),2)+pow((a(2)-b(2)),2));
}

bool write_mesh(const std::string& filename, Pointcloud pointcloud)
{
	float edgeThreshold = 0.001f;

    //number of vertices
	unsigned int nVertices = pointcloud.points.size();

	//number of faces
	unsigned int nFaces = 0;
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

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	//save vertices
    for (int i=0;i<nVertices;i++){
		if (pointcloud.points[i](0)==MINF){
            outFile << "0 0 0 255 255 255 255" << std::endl;
		}
		else{
			outFile << pointcloud.points[i](0) << " " << pointcloud.points[i](1) << " " << pointcloud.points[i](2) << " " <<
			 (int)pointcloud.points[i](3) << " " << (int)pointcloud.points[i](4) << " " << (int)pointcloud.points[i](5) << " " << (int)pointcloud.points[i](6) << std::endl;
		}
	}
	
	// TODO: save faces
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

	// close file
	outFile.close();

    return true;

}
