#include "pose_data_parse.h"

void PoseDataParse::ReadData(std::string file_name) {
    // csv stream
    csvstream csv_in(file_name);
    // log data
    int frame = 0;
    int label = 0;
    int id = 0;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    double detection_score = 0.0;
    double render_score = 0.0;
    std::string img_path = "";

    while(MeasureParse(csv_in, frame, label, id, transform, detection_score, render_score,
    img_path)){
        // We only save on object for each type
        if(id >= 0){
            UpdateObject(id, transform, render_score);
        }
    }

};

Eigen::MatrixXd PoseDataParse::GetData(int object_id){
    Eigen::MatrixXd object_pose = object_poses_[object_id];
    return object_pose;
};

void PoseDataParse::UpdateObject(int object_id, const Eigen::Ref<Eigen::Matrix4d>& pose,
                                 double render_score){
    // if in
    if(render_score < render_score){
        return;
    }

    bool object_existence = true;
    if(object_render_score_.find(object_id) == object_render_score_.end()){
        object_existence = false;
    }

    bool object_update = false;
    if(object_existence){
        if(object_render_score_[object_id] < render_score){
            object_update = true;
        }
    }
    else {
        object_update = true;
    }

    if(object_update){
        object_poses_[object_id] = pose;
    }
};

std::vector<int> PoseDataParse::GetIds(){
    std::vector<int> object_ids;
    object_ids.reserve(object_poses_.size());
    for(auto object : object_poses_){
        object_ids.push_back(object.first);
    }  
    return object_ids;
};