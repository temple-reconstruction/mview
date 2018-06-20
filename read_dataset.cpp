//
// Created by wang yu on 2018/6/10.
//

#include "mview.h"
#include <iostream>
#include <istream>
#include <string>

// read dataset from istream using file
auto read_dataset(std::istream& inputfile) -> std::vector<CameraParameter>{

    std::vector<CameraParameter> cameraParameters;
    std::string line;
    if(!inputfile){
        std::cout<<"cannot open file"<<std::endl;
    }
    std::getline(inputfile,line);
    std::cout<<"imgNum:"<<line<<std::endl;
    while (getline(inputfile,line)){
        CameraParameter cameraParameter;
        std::istringstream iss(line);
        iss>>cameraParameter.filename;
        std::string s;
        std::string ss[21];
        int i=0;
        while (iss>>s){

            ss[i]=s;
            i++;
        }

        cameraParameter.intrinsics(0,0)=std::atof(ss[0].c_str());
        cameraParameter.intrinsics(0,1)=std::atof(ss[1].c_str());
        cameraParameter.intrinsics(0,2)=std::atof(ss[2].c_str());
        cameraParameter.intrinsics(1,0)=std::atof(ss[3].c_str());
        cameraParameter.intrinsics(1,1)=std::atof(ss[4].c_str());
        cameraParameter.intrinsics(1,2)=std::atof(ss[5].c_str());
        cameraParameter.intrinsics(2,0)=std::atof(ss[6].c_str());
        cameraParameter.intrinsics(2,1)=std::atof(ss[7].c_str());
        cameraParameter.intrinsics(2,2)=std::atof(ss[8].c_str());
        cameraParameter.extrinsics(0,0)=std::atof(ss[9].c_str());
        cameraParameter.extrinsics(0,1)=std::atof(ss[10].c_str());
        cameraParameter.extrinsics(0,2)=std::atof(ss[11].c_str());
        cameraParameter.extrinsics(1,0)=std::atof(ss[12].c_str());
        cameraParameter.extrinsics(1,1)=std::atof(ss[13].c_str());
        cameraParameter.extrinsics(1,2)=std::atof(ss[14].c_str());
        cameraParameter.extrinsics(2,0)=std::atof(ss[15].c_str());
        cameraParameter.extrinsics(2,1)=std::atof(ss[16].c_str());
        cameraParameter.extrinsics(2,2)=std::atof(ss[17].c_str());
        cameraParameter.extrinsics(0,3)=std::atof(ss[18].c_str());
        cameraParameter.extrinsics(1,3)=std::atof(ss[19].c_str());
        cameraParameter.extrinsics(2,3)=std::atof(ss[19].c_str());
        cameraParameter.extrinsics(3,0)=0.0;
        cameraParameter.extrinsics(3,1)=0.0;
        cameraParameter.extrinsics(3,2)=0.0;
        cameraParameter.extrinsics(3,3)=1.0;


        cameraParameters.push_back(cameraParameter);
    }

    return cameraParameters;
}
