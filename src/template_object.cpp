#include "template_object.h"

void WriteBox(std::string object_name,
			  const Eigen::Ref<Eigen::VectorXd>& sizes,
			  const Eigen::Ref<Eigen::MatrixXd>& inner_transform)
{

	configuru::Config cfg = configuru::Config::object();
	cfg["object_name"] = object_name;

	double length = sizes(0);
	double width = sizes(1);
	double height = sizes(2);

    // half length representation
    double a = length / 2.0;
    double b =  width / 2.0;
    double c = height / 2.0;

    // save transformation
    cfg["inner_transform"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 0), inner_transform(0, 1), inner_transform(0, 2), inner_transform(0, 3)}),
            configuru::Config::array({inner_transform(1, 0), inner_transform(1, 1), inner_transform(1, 2), inner_transform(1, 3)}),
            configuru::Config::array({inner_transform(2, 0), inner_transform(2, 1), inner_transform(2, 2), inner_transform(2, 3)}),
            configuru::Config::array({inner_transform(3, 0), inner_transform(3, 1), inner_transform(3, 2), inner_transform(3, 3)}),
        }
    );

    cfg["rotation_matrix"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 0), inner_transform(0, 1), inner_transform(0, 2)}),
            configuru::Config::array({inner_transform(1, 0), inner_transform(1, 1), inner_transform(1, 2)}),
            configuru::Config::array({inner_transform(2, 0), inner_transform(2, 1), inner_transform(2, 2)})
        }
    );

    cfg["transition_matrix"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 3)}),
            configuru::Config::array({inner_transform(1, 3)}),
            configuru::Config::array({inner_transform(2, 3)})
        }
    );

	// bascally the gravity center is in the center:
	Eigen::MatrixXd gravity_center(3, 1);
	gravity_center << 0.0,
					  0.0,
					  0.0;

    // log gravity center
	cfg["gravity_center"] = configuru::Config::array({ gravity_center(0),
								                       gravity_center(1), gravity_center(2)});

    // plane feature 
    configuru::Config plane_cfg = configuru::Config::object();

    plane_cfg["plane_approximated"]  = configuru::Config::array(
        {
            false, false, false, false, false, false
        }
    );
	// log normals
    plane_cfg["plane_normals"] = configuru::Config::array(
        {
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({-1, 0, 0}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, -1, 0}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, 0, -1}),
        }
    );

    // plane centers
    plane_cfg["plane_centers"] = configuru::Config::array(
        {
            configuru::Config::array({a , 0, 0}),
            configuru::Config::array({-a, 0, 0}),
            configuru::Config::array({0, b, 0}),
            configuru::Config::array({0, -b, 0}),
            configuru::Config::array({0, 0, c}),
            configuru::Config::array({0, 0, -c}),
        }
    );

    // plane boundary points
    plane_cfg["plane_boundary_points"] = configuru::Config::array(
        {
            configuru::Config::array({a, b, c}),
            configuru::Config::array({a, b, -c}),
            configuru::Config::array({a, -b, -c}),
            configuru::Config::array({a, -b, c}),
            configuru::Config::array({-a, -b, c}),
            configuru::Config::array({-a, b, c}),
            configuru::Config::array({-a, b, -c}),
            configuru::Config::array({-a, -b, -c}),
            configuru::Config::array({a, b, c}),
            configuru::Config::array({a, b, -c}),
            configuru::Config::array({-a, b, -c}),
            configuru::Config::array({-a, b, c}),
            configuru::Config::array({a, -b, -c}),
            configuru::Config::array({a, -b, c}),
            configuru::Config::array({-a, -b, c}),
            configuru::Config::array({-a, -b, -c}),
            configuru::Config::array({a, b, c}),
            configuru::Config::array({a, -b, c}),
            configuru::Config::array({-a, -b, c}),
            configuru::Config::array({-a, b, c}),
            configuru::Config::array({a, b, -c}),
            configuru::Config::array({a, -b, -c}),
            configuru::Config::array({-a, -b, -c}),
            configuru::Config::array({-a, b, -c})
        }
    );

    cfg["plane_feature"] = plane_cfg;

    // cylinder feature
    configuru::Config surf_cfg = configuru::Config::object();
    surf_cfg["surf_approximated"]  = configuru::Config::array(
        {
            false, false, false, false, false, false, 
            false, false, false, false, false, false
        }
    );

    surf_cfg["surf_directions"]  = configuru::Config::array(
        {
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, 0, 1}),
        }
    );

    surf_cfg["surf_centers"]  = configuru::Config::array(
        {
            configuru::Config::array({0, b, c}),
            configuru::Config::array({0, b, -c}),
            configuru::Config::array({0, -b, -c}),
            configuru::Config::array({0, -b, c}),
            configuru::Config::array({-a, 0, c}),
            configuru::Config::array({-a, 0, -c}),
            configuru::Config::array({a, 0, -c}),
            configuru::Config::array({a, 0, c}),
            configuru::Config::array({a, b, 0}),
            configuru::Config::array({a, -b, 0}),
            configuru::Config::array({-a, -b, 0}),
            configuru::Config::array({-a, b, 0}),
        }
    );

    surf_cfg["surf_radius"] =  configuru::Config::array(
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        }
    );

    surf_cfg["surf_boundary_directions"] =  configuru::Config::array(
        {
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, 0, -1}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, -1, 0}),
            configuru::Config::array({0, 0, -1}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, -1, 0}),
            configuru::Config::array({-1, 0, 0}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, 0, -1}),
            configuru::Config::array({-1, 0, 0}),
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({0, 0, -1}),
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({0, -1, 0}),
            configuru::Config::array({1, 0, 0}),
            configuru::Config::array({-1, 0, 0}),
            configuru::Config::array({0, -1, 0}),
            configuru::Config::array({0, 1, 0}),
            configuru::Config::array({-1, 0, 0})
        }
    );

    surf_cfg["surf_boundary_points"] =  configuru::Config::array(
    {
        configuru::Config::array({-a, b, c}),
        configuru::Config::array({a, b, c}),
        configuru::Config::array({-a, b, -c}),
        configuru::Config::array({a, b, -c}),
        configuru::Config::array({-a, -b, -c}),
        configuru::Config::array({a, -b, -c}),
        configuru::Config::array({-a, -b, c}),
        configuru::Config::array({a, -b, c}),
        configuru::Config::array({-a, -b, c}),
        configuru::Config::array({-a, b, c}),
        configuru::Config::array({-a, -b, -c}),
        configuru::Config::array({-a, b, -c}),
        configuru::Config::array({a, -b, -c}),
        configuru::Config::array({a, b, -c}),
        configuru::Config::array({a, -b, c}),
        configuru::Config::array({a, b, c}),
        configuru::Config::array({a, b, -c}),
        configuru::Config::array({a, b, c}),
        configuru::Config::array({a, -b, -c}),
        configuru::Config::array({a, -b, c}),
        configuru::Config::array({-a, -b, -c}),
        configuru::Config::array({-a, -b, c}),
        configuru::Config::array({-a, b, -c}),
        configuru::Config::array({-a, b, c})
    }
    );
    cfg["surf_feature"] = surf_cfg;


    // output
    std::string current_path = GetCurrentWorkingDir();
	configuru::dump_file(current_path + "/models/" + 
                         object_name + ".json", cfg, configuru::JSON);
	
    std::cout << current_path + "/" + object_name + ".json" << std::endl;
};

void WriteCylinder(std::string object_name,
				   const Eigen::Ref<Eigen::VectorXd>& sizes,
				   const Eigen::Ref<Eigen::MatrixXd>& inner_transform)
{
    configuru::Config cfg = configuru::Config::object();
	cfg["object_name"] = object_name;

	double radius = sizes(0);
	double height = sizes(1);

    // half length representation
    double a = radius;
    double b =  height / 2.0;

    cfg["inner_transform"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 0), inner_transform(0, 1), inner_transform(0, 2), inner_transform(0, 3)}),
            configuru::Config::array({inner_transform(1, 0), inner_transform(1, 1), inner_transform(1, 2), inner_transform(1, 3)}),
            configuru::Config::array({inner_transform(2, 0), inner_transform(2, 1), inner_transform(2, 2), inner_transform(2, 3)}),
            configuru::Config::array({inner_transform(3, 0), inner_transform(3, 1), inner_transform(3, 2), inner_transform(3, 3)}),
        }
    );

    // save transformation
    cfg["rotation_matrix"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 0), inner_transform(0, 1), inner_transform(0, 2)}),
            configuru::Config::array({inner_transform(1, 0), inner_transform(1, 1), inner_transform(1, 2)}),
            configuru::Config::array({inner_transform(2, 0), inner_transform(2, 1), inner_transform(2, 2)})
        }
    );

    cfg["transition_matrix"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 3)}),
            configuru::Config::array({inner_transform(1, 3)}),
            configuru::Config::array({inner_transform(2, 3)})
        }
    );
	// bascally the gravity center is in the center:
	Eigen::MatrixXd gravity_center(3, 1);
	gravity_center << 0.0,
					  0.0,
					  0.0;

    // log gravity center
	cfg["gravity_center"] = configuru::Config::array({ gravity_center(0),
								                       gravity_center(1), gravity_center(2)});

    // plane feature 
    configuru::Config plane_cfg = configuru::Config::object();
    plane_cfg["plane_approximated"]  = configuru::Config::array(
        {
            false, false
        }
    );

    // log normals
    plane_cfg["plane_normals"] = configuru::Config::array(
        {
            configuru::Config::array({0, 0, 1}),
            configuru::Config::array({0, 0, -1}),
        }
    );

    // plane centers
    plane_cfg["plane_centers"] = configuru::Config::array(
        {
            configuru::Config::array({0, 0, b}),
            configuru::Config::array({0, 0, -b}),
        }
    );

    plane_cfg["plane_boundary_points"] = configuru::Config::array(
        {
            configuru::Config::array({a, 0, b}),
            configuru::Config::array({0, a, b}),
            configuru::Config::array({-a, 0, b}),
            configuru::Config::array({0, -a, b}),
            configuru::Config::array({a, 0, -b}),
            configuru::Config::array({0, a, -b}),
            configuru::Config::array({-a, 0, -b}),
            configuru::Config::array({0, -a, -b}),
        }
    );
    
    cfg["plane_feature"] = plane_cfg;

    configuru::Config surf_cfg = configuru::Config::object();
    surf_cfg["surf_approximated"]  = configuru::Config::array(
        {
            false
        }
    );

    surf_cfg["surf_directions"]  = configuru::Config::array(
        {
            configuru::Config::array({0, 0, 1})
        }
    );

    surf_cfg["surf_centers"]  = configuru::Config::array(
        {
            configuru::Config::array({0, 0, 0})
        }
    );

    surf_cfg["surf_radius"]  = configuru::Config::array(
        {
            a
        }
    );

    surf_cfg["surf_boundary_directions"]  = configuru::Config::array(
        {
            configuru::Config::array({1, 0.01, 0}),
            configuru::Config::array({1, -0.01, 0}),
        }
    );

    surf_cfg["surf_boundary_points"]  = configuru::Config::array(
        {
            configuru::Config::array({0, 0, -b}),
            configuru::Config::array({0, 0, b}),
        }
    );

    cfg["surf_feature"] = surf_cfg;
    // output
    std::string current_path = GetCurrentWorkingDir();
	configuru::dump_file(current_path + "/models/" + 
                         object_name + ".json", cfg, configuru::JSON);
	
    std::cout << current_path + "/" + object_name + ".json" << std::endl;

};

void WritePlane(std::string object_name,
                const Eigen::Ref<Eigen::VectorXd>& sizes,
                const Eigen::Ref<Eigen::MatrixXd>& inner_transform){
    
    configuru::Config cfg = configuru::Config::object();
	cfg["object_name"] = object_name;

	double length = sizes(0);
	double width = sizes(1);

    // half length representation
    double a = length;
    double b =  width;

    cfg["inner_transform"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 0), inner_transform(0, 1), inner_transform(0, 2), inner_transform(0, 3)}),
            configuru::Config::array({inner_transform(1, 0), inner_transform(1, 1), inner_transform(1, 2), inner_transform(1, 3)}),
            configuru::Config::array({inner_transform(2, 0), inner_transform(2, 1), inner_transform(2, 2), inner_transform(2, 3)}),
            configuru::Config::array({inner_transform(3, 0), inner_transform(3, 1), inner_transform(3, 2), inner_transform(3, 3)}),
        }
    );
    
    // save transformation
    cfg["rotation_matrix"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 0), inner_transform(0, 1), inner_transform(0, 2)}),
            configuru::Config::array({inner_transform(1, 0), inner_transform(1, 1), inner_transform(1, 2)}),
            configuru::Config::array({inner_transform(2, 0), inner_transform(2, 1), inner_transform(2, 2)})
        }
    );

    cfg["transition_matrix"] = configuru::Config::array(
        {
            configuru::Config::array({inner_transform(0, 3)}),
            configuru::Config::array({inner_transform(1, 3)}),
            configuru::Config::array({inner_transform(2, 3)})
        }
    );
	// bascally the gravity center is in the center:
	Eigen::MatrixXd gravity_center(3, 1);
	gravity_center << 0.0,
					  0.0,
					  0.0;

    // log gravity center
	cfg["gravity_center"] = configuru::Config::array({ gravity_center(0),
								                       gravity_center(1), gravity_center(2)});

    // plane feature
    configuru::Config plane_cfg = configuru::Config::object();
    plane_cfg["plane_approximated"]  = configuru::Config::array(
        {
            false
        }
    );

    // log normals
    plane_cfg["plane_normals"] = configuru::Config::array(
        {
            configuru::Config::array({0, 0, 1})
        }
    );

    // plane centers
    plane_cfg["plane_centers"] = configuru::Config::array(
        {
            configuru::Config::array({0, 0, 0})
        }
    );

    plane_cfg["plane_boundary_points"] = configuru::Config::array(
        {
            configuru::Config::array({a, 0, 0}),
            configuru::Config::array({0, b, 0}),
            configuru::Config::array({-a, 0, 0}),
            configuru::Config::array({0, -b, 0})
        }
    );
    
    cfg["plane_feature"] = plane_cfg;

    // output
    std::string current_path = GetCurrentWorkingDir();
	configuru::dump_file(current_path + "/models/" + 
                         object_name + ".json", cfg, configuru::JSON);
	
    std::cout << current_path + "/" + object_name + ".json" << std::endl;
};