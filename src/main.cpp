#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
#include "parse_tool.h"
#include "template_object.h"
#include "geometry_feature.h"

#include <iostream>

#include <Eigen/Eigen>


int main() {
	
    /*
    // Feature Check
	Eigen::MatrixXd plane_normals(3, 3);
	plane_normals << 3.0, 1.0, 2.0,
					 2.0, 3.0, 1.0,
					 -1.0, 2.0, 0.5;
	
	Eigen::MatrixXd plane_centers(3, 3);
	plane_centers << 2.0, 3.0, 1.0,
					 1.0, 2.0, 2.0,
					-1.0, 2.0, 0.5;

	Eigen::MatrixXd plane_boundary_points_1(3, 4);
	plane_boundary_points_1 << 1.0, 2.0, 3.0, 4.0,
							 1.0, 1.0, -1.0, 2.0,
							 0.0, 0.0, 0.0, 1.0;

	Eigen::MatrixXd plane_boundary_points_2(3, 4);
	plane_boundary_points_2 << -1.0, -2.0, -3.0, -4.0,
								1.0, 2.0, 3.0, 4.0,
								0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd surf_boundary_points_1(3, 2);
	surf_boundary_points_1 << -0.3, 4.0,
								1.5, 3.0,
								0.2, 2.0;

	Eigen::MatrixXd surf_boundary_points_2(3, 2);
	surf_boundary_points_2 << -0.2, 2.3,
							  1.3, 1.8,
							  0.6, 0.7;
	
	Eigen::MatrixXd angle_diff(3, 3);
	Eigen::MatrixXd dist_diff(3, 3);

    CalculateDiffPlane2Plane(plane_normals, plane_centers, 
		                     angle_diff, dist_diff);

	// test_2
	CalculateDiffSurf2Surf(plane_normals, plane_centers, dist_diff);
	
	// test_3
	CalculateDiffPlane2Surf(plane_normals, plane_normals,
							plane_centers, plane_centers,
							angle_diff, dist_diff);

	// test_4
	bool consistency = SpatialMatchingCheckV2(plane_boundary_points_1,
											plane_boundary_points_2);

	std::cout << dist_diff << std::endl;

	// test5
	Eigen::Vector3d surf_direction0(1.0, 0, 0.0);
	Eigen::Vector3d surf_direction1(0, 0, 1.0);
	Eigen::Vector3d surf_direction2(0, 1.0, 0.0);
	
	SpatialMatchingCheckV3(
		surf_direction0,
		surf_direction1,
		surf_direction2,
		plane_boundary_points_1,
		surf_boundary_points_1
	);
     */
	
	
	
	// Template Working Check
    
    Eigen::Vector3d box_size(1.0, 2.0, 3.0);
    Eigen::Vector2d cylinder_size(1.0, 2.0);
	Eigen::MatrixXd inner_transfrom(4, 4);
	inner_transfrom << 1.0, 0.0, 0.0, 2.0,
					   0.0, 1.0, 0.0, 3.0,
					   0.0, 0.0, 1.0, 1.0,
					   0.0, 0.0, 0.0, 1.0;
	

    WriteBox("test_box", box_size, inner_transfrom);
    WriteCylinder("test_surf", cylinder_size, inner_transfrom);
    
    // test to scene Inference
    
    std::cout << "template written" << std::endl;
	return 0;

}