#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
#include "parse_tool.h"
#include "template_object.h"
#include "geometry_feature.h"
#include "pose_data_parse.h"
#include "scene_inference.h"

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
    std::string register_config_path = current_path + "/models/object_register.json";
    configuru::Config register_cfg = configuru::parse_file(register_config_path, configuru::JSON);
	
    // get label config
    std::string label_config_path = current_path + "/models/object_label.json";
    configuru::Config label_cfg = configuru::parse_file(label_config_path, configuru::JSON);
    
    // Initial object_register_file
    SceneInference scene_inference(register_config_path);

    // Add objects
    for(auto it : object_ids){
        std::string object_name = (std::string) label_cfg[std::to_string(it)]["name"];
        scene_inference.AddObject(it, object_name);
        
        // update pose data
        Eigen::MatrixXd object_pose = pose_data_parser.GetData(it);
        scene_inference.UpdateObjectPose(it, object_pose);

        if(it == 0){
            // If the object is table & update gravity pose
            scene_inference.SetGravity(object_pose);
            std::cout << "Gravity Pose Update." << std::endl;
        }
            
        // log display
        std::cout << "Label : " << it << " : " << object_name << std::endl;
    }

    // Inference geometry relationship
    scene_inference.RelationshipInference("test_log.json", true);
    // test output
    std::vector<Eigen::MatrixXd> normal_1s;
    std::vector<Eigen::MatrixXd> normal_2s;
    std::vector<Eigen::MatrixXd> center_1s;
    std::vector<Eigen::MatrixXd> center_2s;
    std::vector<double> radius_1s;
    std::vector<double> radius_2s;
    std::vector<int> object_id_1s;
    std::vector<int> object_id_2s;
    std::vector<int> support_types; 
    scene_inference.FeatureForOptimization(
        normal_1s, normal_2s, center_1s, center_2s, 
        radius_1s, radius_2s,
        object_id_1s, object_id_2s, support_types);
        
    return 0;

}
