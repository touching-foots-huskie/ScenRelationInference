#ifndef GEOMETRY_FEATURE
#define GEOMETRY_FEATURE

#include <Eigen/Eigen>

#include <iostream>

#include <cmath>

#include "parse_tool.h"

const double parallel_threshold = 0.01;
const double support_direction_threshold = 0.7;  // 45 degree
const double support_direction_threshold2 = 0.7;

// augment threshold
const double bad_arc_augment_ratio = 0.3;
const double good_arc_augment_ratio = 0.2;
const double cylinder_threshold = 0.9;

// spatial checking should have overlap larger than this
const double overlap_threshold_in_check = 0.2;  

// gravity leverage
const double gravity_leverage_threshold = 0.015;

// Roderegas 
void Rogas(const Eigen::Ref<Eigen::Vector3d>& direction, double theta,
           Eigen::Ref<Eigen::MatrixXd> transform);

void CalculateDiffPlane2Plane(const Eigen::Ref<Eigen::MatrixXd>& plane_normals,
							  const Eigen::Ref<Eigen::MatrixXd>& plane_centeral_points,
							  Eigen::Ref<Eigen::MatrixXd> angle_diff,
							  Eigen::Ref<Eigen::MatrixXd> dist_diff);

void CalculateDiffSurf2Surf(const Eigen::Ref<Eigen::MatrixXd>& surf_directions,
							const Eigen::Ref<Eigen::MatrixXd>& surf_centeral_points,
							Eigen::Ref<Eigen::MatrixXd> dist_diff);

void CalculateDiffPlane2Surf(const Eigen::Ref<Eigen::MatrixXd>& plane_normals,
							 const Eigen::Ref<Eigen::MatrixXd>& surf_directions,
							 const Eigen::Ref<Eigen::MatrixXd>& plane_centeral_points,
							 const Eigen::Ref<Eigen::MatrixXd>& surf_centeral_points,
							 Eigen::Ref<Eigen::MatrixXd> angle_diff,
							 Eigen::Ref<Eigen::MatrixXd> dist_diff); 

// check function for single pairs 
/*
check if the contact normals are supported by existing surface
start direction and end direction and distinguished by r-clock-wise
*/

bool OverlapCheck(double min_1, double min_2, double max_1, double max_2, double overlap_threshold);

bool SurfaceClipCheck(const Eigen::Ref<Eigen::Vector3d>& surf_direction,
					  const Eigen::Ref<Eigen::Vector3d>& start_direction,
					  const Eigen::Ref<Eigen::Vector3d>& end_direction,
					  const Eigen::Ref<Eigen::Vector3d>& normal_direction); 

bool SpatialMatchingCheck(const Eigen::Ref<Eigen::MatrixXd>& boundary_points_1,
						  const Eigen::Ref<Eigen::MatrixXd>& boundary_points_2); 

/*
This function is only used in surface to surface
*/
bool SpatialMatchingCheckV2(const Eigen::Ref<Eigen::MatrixXd>& boundary_points_1,
						    const Eigen::Ref<Eigen::MatrixXd>& boundary_points_2); 

/*
check both if one surface and one plane are matched spatially, and if supporting
structure is enabled
Notice : this only be done when they are geometrically close to each other
boundary_points_1 : boundary points for plane | Dim:[D, K1]
boundary_points_2 : boundary points for surf | Dim:[D, K2]
Usage : Plane2Plane [The first feature should at least have three points].
*/
bool SpatialMatchingCheckV3(const Eigen::Ref<Eigen::Vector3d>& plane_normal,
                            const Eigen::Ref<Eigen::Vector3d>& surf_direction,
							const Eigen::Ref<Eigen::Vector3d>& start_direction,
							const Eigen::Ref<Eigen::Vector3d>& end_direction,
							const Eigen::Ref<Eigen::MatrixXd>& boundary_points_1,
							const Eigen::Ref<Eigen::MatrixXd>& boundary_points_2); 

/*
N1:number of all points, N2:number of projection norms
check if the gravity center is supported by a combination of several features
boudary_points | Dim:[D, N1]
gravity_center | Dim:[D, 1]
projection_norms | Dim:[D, N2]

*/
bool GravitySupportCheck(const Eigen::Ref<Eigen::MatrixXd>& boundary_points,
						 const Eigen::Ref<Eigen::MatrixXd>& gravity_center,
						 const Eigen::Ref<Eigen::MatrixXd>& projection_norms); 

/*
Check if the normals of planes follows supporting with gravity
Used in plane Feature
plane_normals | Dim:[D, N]
gravity_direction | Dim:[D, 1]
supporting_status | Dim:[1, N]
*/
void SupportingStatus(const Eigen::Ref<Eigen::MatrixXd>& plane_normals,
					  const Eigen::Ref<Eigen::MatrixXd>& gravity_direction,
					  Eigen::Ref<Eigen::MatrixXd> supporting_status); 

/*
Used for surface
surf_directions | Dim:[D, N]
gravity_direction | Dim:[D, 1]
supporting_status | Dim:[1, N]
*/
void SupportingStatusV2(const Eigen::Ref<Eigen::MatrixXd>& surf_directions,
						const Eigen::Ref<Eigen::MatrixXd>& gravity_direction,
						Eigen::Ref<Eigen::MatrixXd> supporting_status);


#endif