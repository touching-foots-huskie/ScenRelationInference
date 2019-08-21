#include "geometry_feature.h"

void Rogas(const Eigen::Ref<Eigen::Vector3d>& direction, double theta,
      Eigen::Ref<Eigen::MatrixXd> transform) {
        transform = Eigen::Matrix3d::Identity();
        transform *= cos(theta);

        transform += (1 - cos(theta)) * (direction * direction.transpose());

        Eigen::Matrix3d d_upper;
        d_upper << 0, -direction(2), direction(1),
                   direction(2), 0, -direction(0),
                   -direction(1), direction(0), 0;

        transform += sin(theta) * d_upper;
};

void CalculateDiffPlane2Plane(const Eigen::Ref<Eigen::MatrixXd>& plane_normals,
                              const Eigen::Ref<Eigen::MatrixXd>& plane_centeral_points,
                              Eigen::Ref<Eigen::MatrixXd> angle_diff,
                              Eigen::Ref<Eigen::MatrixXd> dist_diff) {

	int D = plane_normals.rows();
	int N = plane_normals.cols();

	angle_diff = Eigen::MatrixXd::Ones(N, N) + plane_normals.transpose() * plane_normals;
	angle_diff /= 2.0;  // rescale to [0, 1]

	// dist_diff
	Eigen::MatrixXd rep_vectors_1 = plane_centeral_points.replicate(N, 1);
	rep_vectors_1.resize(D, N * N);
	Eigen::MatrixXd rep_vectors_2 = plane_centeral_points.transpose().replicate(N, 1);
	Eigen::MatrixXd diff_vectors = -rep_vectors_1 + rep_vectors_2.transpose();  // Dim:[D, N*N]
	Eigen::MatrixXd aug_dist_diff = plane_normals.transpose() * diff_vectors;  // Dim:[N, N*N]
	dist_diff = Eigen::MatrixXd::Zero(N, N);

	for (int i = 0; i < N; i++) {
		for (int j = i + 1; j < N; j++) {
			dist_diff(i, j) = aug_dist_diff(i, i * N + j);
			dist_diff(j, i) = dist_diff(i, j);
		}
	}

    // absolute value
    dist_diff = dist_diff.cwiseAbs();
};

void CalculateDiffSurf2Surf(const Eigen::Ref<Eigen::MatrixXd>& surf_directions,
	                        const Eigen::Ref<Eigen::MatrixXd>& surf_centeral_points,
	                        Eigen::Ref<Eigen::MatrixXd> dist_diff) {
	
    int D = surf_directions.rows();
	int N = surf_directions.cols();

	Eigen::MatrixXd rep_vectors_1 = surf_centeral_points.replicate(N, 1);
	rep_vectors_1.resize(D, N * N);
	Eigen::MatrixXd rep_vectors_2 = surf_centeral_points.replicate(1, N);
	Eigen::MatrixXd diff_vectors = -rep_vectors_1 + rep_vectors_2;  // Dim:[D, N*N]

	dist_diff = Eigen::MatrixXd::Zero(N, N);
	for (int i = 0; i < N; i++) {
		for (int j = i + 1; j < N; j++) {
			Eigen::Vector3d direction_1 = surf_directions.col(i);
			Eigen::Vector3d direction_2 = surf_directions.col(j);
			Eigen::Vector3d co_normal = direction_1.cross(direction_2);
			double normal_len = co_normal.norm();
			Eigen::Vector3d diff_vector = diff_vectors.col(i * N + j);
			if (normal_len < parallel_threshold) {
				double diff_vector_norm = diff_vector.norm();

				double diff_vector_proj = direction_1.dot(diff_vector);
				dist_diff(i, j) = sqrt(pow(diff_vector_norm, 2) - pow(diff_vector_proj, 2));
			}
			else {
				co_normal /= normal_len;
				dist_diff(i, j) = abs(co_normal.dot(diff_vector));
			}
			dist_diff(j, i) = dist_diff(i, j);
		}
	}

    dist_diff = dist_diff.cwiseAbs();
};

/*
N1 is the number of planes, N2 is the number of surfs
plane_centeral_points | Dim:[D, N1]
surf_centeral_points | Dim:[D, N2]
 */
void CalculateDiffPlane2Surf(const Eigen::Ref<Eigen::MatrixXd>& plane_normals,
                             const Eigen::Ref<Eigen::MatrixXd>& surf_directions,
                             const Eigen::Ref<Eigen::MatrixXd>& plane_centeral_points,
                             const Eigen::Ref<Eigen::MatrixXd>& surf_centeral_points,
                             Eigen::Ref<Eigen::MatrixXd> angle_diff,
                             Eigen::Ref<Eigen::MatrixXd> dist_diff) {
	
    int D = plane_normals.rows();
	int N1 = plane_normals.cols();
	int N2 = surf_directions.cols();
	angle_diff = plane_normals.transpose() * surf_directions;
	angle_diff = angle_diff.cwiseAbs();

	Eigen::MatrixXd rep_vectors_1 = plane_centeral_points.replicate(N2, 1);
	rep_vectors_1.resize(D, N1 * N2);
	Eigen::MatrixXd rep_vectors_2 = surf_centeral_points.replicate(1, N1);
	Eigen::MatrixXd diff_vectors = rep_vectors_1 - rep_vectors_2;  // Dim:[D, N1*N2]
	Eigen::MatrixXd aug_dist_diff = plane_normals.transpose() * diff_vectors;  // Dim:[N1, N1*N2]

	dist_diff = Eigen::MatrixXd::Zero(N1, N2);

    // Plane2Surf is a one-direction looping structure
	for (int i = 0; i < N1; i++) {
		for (int j = 0; j < N2; j++) {
			dist_diff(i, j) = aug_dist_diff(i, i * N2 + j);
		}
	}

    dist_diff = dist_diff.cwiseAbs();
};

// check function for single pairs 
/*
check if the contact normals are supported by existing surface
start direction and end direction and distinguished by r-clock-wise
*/

bool SurfaceClipCheck(const Eigen::Ref<Eigen::Vector3d>& surf_direction,
                      const Eigen::Ref<Eigen::Vector3d>& start_direction,
                      const Eigen::Ref<Eigen::Vector3d>& end_direction,
                      const Eigen::Ref<Eigen::Vector3d>& normal_direction) {

	Eigen::Vector3d startp90 = surf_direction.cross(start_direction);
	Eigen::Vector3d endp90 = surf_direction.cross(end_direction);

    // if cylinder first & return true clip
    double direction_gap = start_direction.dot(end_direction);
    if(direction_gap > cylinder_threshold){
        return true;
    }

	double arc_dot = startp90.dot(end_direction);
	bool clip_status;

    double start_status = startp90.dot(normal_direction);
    double end_status = endp90.dot(normal_direction);

    // add different augment based on different arc type

	if (arc_dot >= 0) {
		// bad arc
		clip_status = false;
		if ((start_status < bad_arc_augment_ratio) &&
             (end_status >= -bad_arc_augment_ratio))
			clip_status = true;
	}
	else
	{
		// good arc : actual
		clip_status = true;
		if ((start_status >= good_arc_augment_ratio) &&
             (end_status < good_arc_augment_ratio))
			clip_status = false;
	}
	return clip_status;
};

bool OverlapCheck(double min_1, double min_2, double max_1, double max_2, double overlap_threshold){
    double eps = 1e-5;
    if(max_1 > max_2){
        if(min_1 > max_2){
            return false;
        }
        else
        {
            double overlap_value = max_2 - min_1;
            double overlap_ratio = std::max(overlap_value / (max_1 - min_1 + eps),
                 overlap_value / (max_2 - min_2 + eps));
            if(overlap_ratio <= overlap_threshold){
                return false;
            }
            else{
                return true;
            }
        }
        
    }
    else
    {
        if(min_2 > max_1){
            return false;
        }
        else
        {
            double overlap_value = max_1 - min_2;
            double overlap_ratio = std::max(overlap_value / (max_2 - min_2 + eps),
                 overlap_value / (max_1 - min_1 + eps));
            if(overlap_ratio <= overlap_threshold){
                return false;
            }
            else{
                return true;
            }
        }
    }
    
};

/*
For all spatial matching checking, a overlapping threshold is added
 */
bool SpatialMatchingCheck(const Eigen::Ref<Eigen::MatrixXd>& boundary_points_1,
	                      const Eigen::Ref<Eigen::MatrixXd>& boundary_points_2) {

	int K1 = boundary_points_1.cols();
	int K2 = boundary_points_2.cols();

	bool matched = true;

	Eigen::Vector3d edge_k_0 = boundary_points_1.col(0) - boundary_points_1.col(1);
	Eigen::Vector3d edge_k_1 = boundary_points_1.col(1) - boundary_points_1.col(2);
	Eigen::Vector3d normal = edge_k_0.cross(edge_k_1);
	
	if (normal.norm() != 0) {
        normal /= normal.norm();
		for (int k = 0; k < K1 - 1; k++) {
			Eigen::Vector3d edge_k = boundary_points_1.col(k) - boundary_points_1.col(k + 1);//:[D,1]
			Eigen::MatrixXd normal_k = normal.cross(edge_k);
			Eigen::MatrixXd projection_i = normal_k.transpose() * boundary_points_1; // Dim:[1,K]
			Eigen::MatrixXd projection_j = normal_k.transpose() * boundary_points_2;
			
            double max_1 = projection_i.maxCoeff();
            double max_2 = projection_j.maxCoeff();
            double min_1 = projection_i.minCoeff();
            double min_2 = projection_j.minCoeff();
            if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
                return false;
            }
		}

		for (int k = 0; k < K2 - 1; k++) {
			Eigen::Vector3d edge_k = boundary_points_2.col(k) - boundary_points_2.col(k + 1);//:[D,1]
			Eigen::MatrixXd normal_k = normal.cross(edge_k);
			Eigen::MatrixXd projection_i = normal_k.transpose() * boundary_points_1; // Dim:[1,K]
			Eigen::MatrixXd projection_j = normal_k.transpose() * boundary_points_2;
			double max_1 = projection_i.maxCoeff();
            double max_2 = projection_j.maxCoeff();
            double min_1 = projection_i.minCoeff();
            double min_2 = projection_j.minCoeff();
            if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
                return false;
            }
		}

	}
	else {
		std::cout << "This is not the boundary points of plane, but a line" << std::endl;
		assert(false);
	}

	return matched;

};

/*
This function is only used in surface to surface
*/
bool SpatialMatchingCheckV2(const Eigen::Ref<Eigen::MatrixXd>& boundary_points_1,
	                        const Eigen::Ref<Eigen::MatrixXd>& boundary_points_2) {
	// TODO: Add surface clip check here

	bool matched = true;

	Eigen::Vector3d edge_k_0 = boundary_points_1.col(0) - boundary_points_1.col(1);
	Eigen::Vector3d edge_k_1 = boundary_points_2.col(0) - boundary_points_2.col(1);
	Eigen::Vector3d normal = edge_k_0.cross(edge_k_1);

	if (normal.norm() != 0) {
        normal /= normal.norm();
		for (int k = 0; k < 1; k++) {
			Eigen::Vector3d edge_k = boundary_points_1.col(k) - boundary_points_1.col(k + 1);//:[D,1]
			Eigen::Vector3d normal_k = normal.cross(edge_k);
			Eigen::MatrixXd projection_i = normal_k.transpose() * boundary_points_1; // Dim:[1,K]
			Eigen::MatrixXd projection_j = normal_k.transpose() * boundary_points_2;
			double max_1 = projection_i.maxCoeff();
            double max_2 = projection_j.maxCoeff();
            double min_1 = projection_i.minCoeff();
            double min_2 = projection_j.minCoeff();
            if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
                return false;
            }
		}

		for (int k = 0; k < 1; k++) {
			Eigen::Vector3d edge_k = boundary_points_2.col(k) - boundary_points_2.col(k + 1);//:[D,1]
			Eigen::Vector3d normal_k = normal.cross(edge_k);
			Eigen::MatrixXd projection_i = normal_k.transpose() * boundary_points_1; // Dim:[1,K]
			Eigen::MatrixXd projection_j = normal_k.transpose() * boundary_points_2;
			double max_1 = projection_i.maxCoeff();
            double max_2 = projection_j.maxCoeff();
            double min_1 = projection_i.minCoeff();
            double min_2 = projection_j.minCoeff();
            if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
                return false;
            }
		}
	}
	else {
		// If by any chance, the two surface are parallel to each other
		for (int k = 0; k < 1; k++) {
			Eigen::MatrixXd edge_k = boundary_points_1.col(k) - boundary_points_1.col(k + 1);//:[D,1]
			Eigen::MatrixXd projection_i = edge_k.transpose() * boundary_points_1; // Dim:[1,K]
			Eigen::MatrixXd projection_j = edge_k.transpose() * boundary_points_2;
			double max_1 = projection_i.maxCoeff();
            double max_2 = projection_j.maxCoeff();
            double min_1 = projection_i.minCoeff();
            double min_2 = projection_j.minCoeff();
            if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
                return false;
            }
		}

		for (int k = 0; k < 1; k++) {
			Eigen::MatrixXd edge_k = boundary_points_2.col(k) - boundary_points_2.col(k + 1);//:[D,1]
			Eigen::MatrixXd projection_i = edge_k.transpose() * boundary_points_1; // Dim:[1,K]
			Eigen::MatrixXd projection_j = edge_k.transpose() * boundary_points_2;
			double max_1 = projection_i.maxCoeff();
            double max_2 = projection_j.maxCoeff();
            double min_1 = projection_i.minCoeff();
            double min_2 = projection_j.minCoeff();
            if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
                return false;
            }
		}
	}

	return matched;
};

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
	                        const Eigen::Ref<Eigen::MatrixXd>& boundary_points_2) {

	int K1 = boundary_points_1.cols();
	int K2 = boundary_points_2.cols();

	bool matched = true;

    // Get surface normal
	Eigen::Vector3d normal = plane_normal;

	assert(normal.norm() != 0);

    normal /= normal.norm();
	for (int k = 0; k < K1 - 1; k++) {
		Eigen::Vector3d edge_k = boundary_points_1.col(k) - boundary_points_1.col(k + 1);//:[D,1]
		Eigen::Vector3d normal_k = normal.cross(edge_k);
		Eigen::MatrixXd projection_i = normal_k.transpose() * boundary_points_1; // Dim:[1,K]
		Eigen::MatrixXd projection_j = normal_k.transpose() * boundary_points_2;
		double max_1 = projection_i.maxCoeff();
        double max_2 = projection_j.maxCoeff();
        double min_1 = projection_i.minCoeff();
        double min_2 = projection_j.minCoeff();
        if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
            return false;
        }
	}

	for (int k = 0; k < K2 - 1; k++) {
		Eigen::Vector3d edge_k = boundary_points_2.col(k) - boundary_points_2.col(k + 1);//:[D,1]
		Eigen::Vector3d normal_k = normal.cross(edge_k);
		Eigen::MatrixXd projection_i = normal_k.transpose() * boundary_points_1; // Dim:[1,K]
		Eigen::MatrixXd projection_j = normal_k.transpose() * boundary_points_2;
		double max_1 = projection_i.maxCoeff();
        double max_2 = projection_j.maxCoeff();
        double min_1 = projection_i.minCoeff();
        double min_2 = projection_j.minCoeff();
        if(!OverlapCheck(min_1, min_2, max_1, max_2, overlap_threshold_in_check)){
            return false;
        }
	}

	if (matched) {
		matched = SurfaceClipCheck(surf_direction, start_direction, end_direction, normal);
	}

	return matched;

};

/*
N1:number of all points, N2:number of projection norms
check if the gravity center is supported by a combination of several features
boudary_points | Dim:[D, N1]
gravity_center | Dim:[D, 1]
projection_norms | Dim:[D, N2]

*/
bool GravitySupportCheck(const Eigen::Ref<Eigen::MatrixXd>& boundary_points,
	                     const Eigen::Ref<Eigen::MatrixXd>& gravity_center,
	                     const Eigen::Ref<Eigen::MatrixXd>& projection_norms) {
	                     Eigen::MatrixXd projected_centers = gravity_center.transpose() * projection_norms;  // [1, N2]
	                     Eigen::MatrixXd projected_points = boundary_points.transpose() * projection_norms;  // [N1, N2]

	int N2 = projection_norms.cols();
	for (int i = 0; i < N2; i++) {
		Eigen::MatrixXd projected_points_i = projected_points.col(i);
		if (projected_points_i.maxCoeff() < projected_centers(i) ||
			projected_points_i.minCoeff() > projected_centers(i))
		{
			return false;
		}
	}
	return true;
};

/*
Check if the normals of planes follows supporting with gravity
Used in plane Feature
plane_normals | Dim:[D, N]
gravity_direction | Dim:[D, 1]
supporting_status | Dim:[1, N]
*/
void SupportingStatus(const Eigen::Ref<Eigen::MatrixXd>& plane_normals,
                      const Eigen::Ref<Eigen::MatrixXd>& gravity_direction,
                      Eigen::Ref<Eigen::MatrixXd> supporting_status) {
                      
	supporting_status = gravity_direction.transpose() * plane_normals;  // Dim:[1, N]

	for (int i = 0; i < supporting_status.cols(); i++) {
		double plane_angle = supporting_status(0, i);
		if (plane_angle >= support_direction_threshold)
		{
			supporting_status(0, i) = 1.0;
		}
		else if (plane_angle <= -support_direction_threshold) {
			supporting_status(0, i) = -1.0;
		}
		else if ((plane_angle < support_direction_threshold) &&
			(plane_angle > 0)) {
			supporting_status(0, i) = 0.5;
		}
		else if ((plane_angle > -support_direction_threshold) &&
			(plane_angle < 0)) {
			supporting_status(0, i) = -0.5;
		}
	}
};

/*
Used for surface
surf_directions | Dim:[D, N]
gravity_direction | Dim:[D, 1]
supporting_status | Dim:[1, N]
*/
void SupportingStatusV2(const Eigen::Ref<Eigen::MatrixXd>& surf_directions,
	                    const Eigen::Ref<Eigen::MatrixXd>& gravity_direction,
	                    Eigen::Ref<Eigen::MatrixXd> supporting_status) {
	
    supporting_status = gravity_direction.transpose() * surf_directions;
    // DisplayMatrix(supporting_status, "Supporting_Status");

	double surf_threshold = support_direction_threshold2;
	// Supporting Feature
	for (int i = 0; i < supporting_status.cols(); i++) {
		double plane_angle = supporting_status(0, i);
		if (abs(plane_angle) <= surf_threshold) {
			supporting_status(0, i) = 1.0;
		}
		else {
			supporting_status(0, i) = 0.0;
		}
	}
};
