#include "template_object.h"

void WriteBox(std::string object_name,
			  const Eigen::Ref<Eigen::Vector3d>& sizes,
			  const Eigen::Ref<Eigen::MatrixXd>& inner_transform)
{
	/*
	configuru::Config cfg = configuru::Config::object();
	cfg["object_name"] = object_name;

	double length = sizes(0);
	double width = sizes(1);
	double height = sizes(2);

	Eigen::Matrix3d rotation_matrix = inner_transform.block(0, 0, 3, 3);
	Eigen::MatrixXd transition_matrix = inner_transform.block(0, 3, 3, 1);

	// bascally the gravity center is in the center:
	Eigen::MatrixXd gravity_center(3, 1);
	gravity_center << 0.0,
					  0.0,
					  0.0;
	gravity_center = rotation_matrix * gravity_center + transition_matrix;

	// log gravity center
	// cfg["gravity_center"] = configuru::Config::array({ gravity_center(0),
	//							              gravity_center(1), gravity_center(2)});

	// output
	configuru::dump_file(object_name + ".json", cfg, configuru::JSON);
	*/
	
};

void WriteCylinder(std::string object_name,
				   const Eigen::Ref<Eigen::Vector3d>& sizes,
				   const Eigen::Ref<Eigen::MatrixXd>& inner_transfrom)
{
};
