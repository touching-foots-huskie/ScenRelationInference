#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
#include "parse_tool.h"
#include "template_object.h"
#include "geometry_feature.h"
#include "pose_data_parse.h"

#include <iostream>

#include <Eigen/Eigen>


int main() {

    PoseDataParse pose_data_parser;

    std::string current_path = GetCurrentWorkingDir();
    std::string file_name = "0630.csv";
    pose_data_parser.ReadData(current_path + "/data/" + file_name);
    std::vector<int> object_ids = pose_data_parser.GetIds();
    
    // construct objects based on file
    // get register config
    
	return 0;

}