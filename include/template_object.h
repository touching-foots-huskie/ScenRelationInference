#ifndef template_object
#define template_object

#include "parse_tool.h"
#include "configuru.hpp"

#include <Eigen/Eigen>

#include <vector>
#include <iostream>
/*
template object can generate config file for different pre-defined geometry objects
*/
void WriteBox(std::string object_name, 
			  const Eigen::Ref<Eigen::Vector3d>& sizes,
			  const Eigen::Ref<Eigen::MatrixXd>& inner_transform);

void WriteCylinder(std::string object_name,  
				   const Eigen::Ref<Eigen::Vector2d>& sizes,
				   const Eigen::Ref<Eigen::MatrixXd>& inner_transfrom);

#endif // !template_object

