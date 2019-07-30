#ifndef POSE_DATA_PARSE_H
#define POSE_DATA_PARSE_H

#include "data_parse.hpp"
#include "csvstream.hpp"

#include <Eigen/Eigen>

#include <vector>

const double render_threshold = 0.2;

class PoseDataParse{
public:
    // get the best one in one type
    void ReadData(std::string file_name);
    Eigen::MatrixXd GetData(int object_id);
    void UpdateObject(int object_id, const Eigen::Ref<Eigen::Matrix4d>& pose,
                      double render_score);
    std::vector<int> GetIds();
private:
    std::map<int, Eigen::MatrixXd> object_poses_;
    std::map<int, double> object_render_score_;
};

#endif