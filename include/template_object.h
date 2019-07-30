#ifndef TEMPLATE_OBJECT
#define TEMPLATE_OBJECT

#include "parse_tool.h"
#include "configuru.hpp"

#include <Eigen/Eigen>

#include <vector>
#include <iostream>
/*
template object can generate config file for different pre-defined geometry objects
*/
void WriteBox(std::string object_name, 
			  const Eigen::Ref<Eigen::VectorXd>& sizes,
			  const Eigen::Ref<Eigen::MatrixXd>& inner_transform);

void WriteCylinder(std::string object_name,  
				   const Eigen::Ref<Eigen::VectorXd>& sizes,
				   const Eigen::Ref<Eigen::MatrixXd>& inner_transform);

void WritePlane(std::string object_name,
                const Eigen::Ref<Eigen::VectorXd>& sizes,
                const Eigen::Ref<Eigen::MatrixXd>& inner_transform);

#endif // !template_object

