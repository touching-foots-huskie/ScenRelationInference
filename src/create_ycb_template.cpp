#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"

#include "template_object.h"
#include "parse_tool.h"

int main(){
    // parse object label
    std::string current_path = GetCurrentWorkingDir();
    std::string label_config_path = current_path + "/models/object_label.json";
    std::string register_config_path = current_path + "/models/object_register.json";
    
    configuru::Config label_cfg = configuru::parse_file(label_config_path, configuru::JSON);
    configuru::Config register_cfg = configuru::parse_file(register_config_path, configuru::JSON);
    // parse and creates file in models/
    for(auto& label : label_cfg.as_object()){
        configuru::Config object_cfg = label.value();
        std::string object_name = (std::string) object_cfg["name"];
        Eigen::VectorXd object_size = VetorParse(object_cfg["size"]);
        int template_type = (int) object_cfg["type"];
        Eigen::MatrixXd inner_transform = MatrixParse(object_cfg["inner_transform"]);
        switch (template_type)
        {
        case 0:
            WritePlane(object_name, object_size, inner_transform);
            break;
        case 1:
            WriteBox(object_name, object_size, inner_transform);
            break;
        case 2:
            WriteCylinder(object_name, object_size, inner_transform);
            break;
        default:
            break;
        }
        // log it into register.json
        register_cfg[object_name] = object_name + ".json";
        std::cout << object_name << " template written." << std::endl;
    }

    // write log register
    configuru::dump_file(register_config_path, register_cfg, configuru::JSON);
    return 0;
}