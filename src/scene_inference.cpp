#include "scene_inference.h"

//method 
void ProjectionFeatureManager::AddFeature(int feature_id,
                    const Eigen::Ref<Eigen::MatrixXd>& feature_boundary_points_1,
		            const Eigen::Ref<Eigen::MatrixXd>& feature_boundary_points_2) {
	
	// pairs 1
    int N1 = feature_boundary_points_1.cols();

	Eigen::MatrixXd projected_boundary_points_1 = rotation_transform_ * feature_boundary_points_1
		+ transition_transform_.replicate(1, N1);
	
	projected_boundary_points_1.block(2, 0, 1, N1) *= 0;  // Remove z axis

	// pairs 2
    int N2 = feature_boundary_points_2.cols();

	Eigen::MatrixXd projected_boundary_points_2 = rotation_transform_ * feature_boundary_points_2
		+ transition_transform_.replicate(1, N2);
	
	projected_boundary_points_2.block(2, 0, 1, N2) *= 0;  // Remove z axis

	// add index into feature index
	points_1_index_feature_[feature_id].push_back(boundary_points_1_.cols());
	points_1_index_feature_[feature_id].push_back(N1);
    points_2_index_feature_[feature_id].push_back(boundary_points_2_.cols());
	points_2_index_feature_[feature_id].push_back(N2);

	norms_index_feature_[feature_id].push_back(projection_norms_.cols());
	norms_index_feature_[feature_id].push_back(N1 + N2);

	// add the proected data into projected boundary points & edges
	ConcatMatrix(boundary_points_1_, projected_boundary_points_1);
	ConcatMatrix(boundary_points_2_, projected_boundary_points_2);

	// z_axis is this, because this is what gravity transform is doing
	Eigen::Vector3d z_axis(0, 0, 1);
	// edges1
	Eigen::MatrixXd pro_normals = Eigen::MatrixXd::Zero(3, N1 + N2);

	Eigen::MatrixXd pro_edges_1 = projected_boundary_points_1.block(0, 0, 3, N1 - 1)
		- projected_boundary_points_1.block(0, 1, 3, N1 - 1);

	for (int i = 0; i < N1 - 1; i++) {
		Eigen::Vector3d this_edge = pro_edges_1.block(0, i, 3, 1);
		pro_normals.block(0, i, 3, 1) = this_edge.cross(z_axis);
	}

	// col start from 0
	Eigen::Vector3d last_edge_1 = projected_boundary_points_1.col(N1 - 1) -
		projected_boundary_points_1.col(0);

	pro_normals.block(0, N1 - 1, 3, 1) = last_edge_1.cross(z_axis);

	// edges2
	Eigen::MatrixXd pro_normals_2 = projected_boundary_points_2.block(0, 0, 3, N2 - 1)
		- projected_boundary_points_2.block(0, 1, 3, N2 - 1);

	for (int i = 0; i < N2 - 1; i++) {
		Eigen::Vector3d this_edge = pro_normals_2.block(0, i, 3, 1);
		pro_normals.block(0, N1 + i, 3, 1) = this_edge.cross(z_axis);
	}

	// col start from 0
	Eigen::Vector3d last_edge_2 = projected_boundary_points_2.col(N2 - 1) -
		projected_boundary_points_2.col(0);

	pro_normals.block(0, N1 + N2 - 1, 3, 1) = last_edge_2.cross(z_axis);

	ConcatMatrix(projection_norms_, pro_normals);
};

void ProjectionFeatureManager::RemoveFeature(int feature_id) {
	// Remove feature id is to remove related feature
	int points_1_start_index = points_1_index_feature_[feature_id][0];
	int points_1_feature_num = points_1_index_feature_[feature_id][1];
    int points_2_start_index = points_2_index_feature_[feature_id][0];
	int points_2_feature_num = points_2_index_feature_[feature_id][1];

	int norms_start_index = norms_index_feature_[feature_id][0];
	int norms_feature_num = norms_index_feature_[feature_id][1];

	// make a copy first
	copy_boundary_points_1_ = boundary_points_1_;
    copy_boundary_points_2_ = boundary_points_2_;
	copy_projection_norms_ = projection_norms_;

	// Remove then
	RemoveMatrix(boundary_points_1_, points_1_start_index, points_1_feature_num);
    RemoveMatrix(boundary_points_2_, points_2_start_index, points_2_feature_num);
	RemoveMatrix(projection_norms_, norms_start_index, norms_feature_num);

	points_1_index_feature_.erase(feature_id);
    points_2_index_feature_.erase(feature_id);
	norms_index_feature_.erase(feature_id);
};

void ProjectionFeatureManager::RecoverLastRemove() {
    // TODO: if we need to add feature index back
    // Seems needless
	boundary_points_1_ = copy_boundary_points_1_;
    boundary_points_2_ = copy_boundary_points_2_;
	projection_norms_ = copy_projection_norms_;
};

bool ProjectionFeatureManager::SupportingStatus() {
	// exam if current points & normals can support gravity center
	bool supported_1 = GravitySupportCheck(boundary_points_1_,
		gravity_center_, projection_norms_);
    bool supported_2 = GravitySupportCheck(boundary_points_2_,
		gravity_center_, projection_norms_);
	return (supported_1 && supported_2);
};

SceneInference::SceneInference(std::string object_register_file)
{
	num_of_object_ = 0;
	num_of_plane_ = 0;
	num_of_surf_ = 0;

	object_register_file_ = object_register_file;
	object_register_config_ = configuru::parse_file(object_register_file_, configuru::JSON);

    // file path
    current_working_path_ = GetCurrentWorkingDir();
};

bool SceneInference::IsObjectIn(int object_id) {
	if (object_deprecated_.find(object_id) == object_deprecated_.end())
		return false;
	else
		return true;
};

void SceneInference::AddObject(int object_id, std::string object_name) {

	num_of_object_++;
	current_object_id_ = object_id;
	object_deprecated_[object_id] = false;  // Not deprecated
    object_names_[object_id] = object_name;

	std::string object_config_file = current_working_path_ + "/models/" + 
                                    (std::string) object_register_config_[object_name];
	
    configuru::Config geometry_config = configuru::parse_file(object_config_file,
		configuru::JSON);

	object_name = (std::string) geometry_config["object_name"];

    // Inner Transform
    object_inner_transform_[object_id] = MatrixParse(geometry_config["inner_transform"]).inverse();
    Eigen::MatrixXd rotation_matrix = object_inner_transform_[object_id].block(0, 0, 3, 3);
    Eigen::MatrixXd transition_matrix = object_inner_transform_[object_id].block(0, 3, 3, 1);

    // gravity center
	Eigen::MatrixXd  gravity_center = VetorParse(geometry_config["gravity_center"]);
	object_gravity_centers_[object_id] = rotation_matrix * gravity_center 
        + transition_matrix;

	// Add Plane if plane in configuration 
    if(geometry_config.has_key("plane_feature")){
        configuru::Config plane_config = geometry_config["plane_feature"];
	    AddPlaneFeature(plane_config);
    }
	
    // Surf
    if(geometry_config.has_key("surf_feature")){
        configuru::Config surf_config = geometry_config["surf_feature"];
	    AddSurfFeature(surf_config);
    }

	// initialize object pose
    object_poses_[object_id] = Eigen::MatrixXd::Identity(4, 4);
};

void SceneInference::AddPlaneFeature(configuru::Config& plane_config) {
	
    Eigen::MatrixXd rotation_transform = object_inner_transform_[current_object_id_].block(0, 0, 3, 3);
    Eigen::MatrixXd transition_transform = object_inner_transform_[current_object_id_].block(0, 3, 3, 1);

    Eigen::MatrixXd plane_normal_object = MatrixParseT(plane_config["plane_normals"]);
	
    // plane normal
    int num_of_plane_object = plane_normal_object.cols();
    plane_normal_object = rotation_transform * plane_normal_object;
    ConcatMatrix(plane_normals_, plane_normal_object);
    ConcatMatrix(opt_plane_normals_, plane_normal_object);

    // plane center
	Eigen::MatrixXd plane_center_object = MatrixParseT(plane_config["plane_centers"]);
    plane_center_object = rotation_transform * plane_center_object +
        transition_transform.replicate(1, num_of_plane_object);
	ConcatMatrix(plane_central_points_, plane_center_object);
    ConcatMatrix(opt_plane_central_points_, plane_center_object);

    // plane boundary points
	Eigen::MatrixXd plane_boundary_points = MatrixParseT(plane_config["plane_boundary_points"]);
    plane_boundary_points = rotation_transform * plane_boundary_points +
        transition_transform.replicate(1, num_of_plane_object * 4);
    ConcatMatrix(plane_boundary_points_, plane_boundary_points);

	plane_feature_index_object_[current_object_id_].clear();

	for (unsigned int index = num_of_plane_; index < (num_of_plane_ + num_of_plane_object); index++) {
		// assert
		assert(feature_deprecated_["plane"].size() == index);
		feature_deprecated_["plane"].push_back(false);
		plane_feature2id.push_back(current_object_id_);
		plane_feature_index_object_[current_object_id_].push_back(index);

	}
	num_of_plane_ += num_of_plane_object;

	for (auto v : plane_config["plane_approximated"].as_array()) {
		feature_approximated_["plane"].push_back((bool)v);
	}
};

void SceneInference::AddSurfFeature(configuru::Config& surf_config) {
    Eigen::MatrixXd rotation_transform = object_inner_transform_[current_object_id_].block(0, 0, 3, 3);
    Eigen::MatrixXd transition_transform = object_inner_transform_[current_object_id_].block(0, 3, 3, 1);

    // surf directions
	Eigen::MatrixXd surf_directions = MatrixParseT(surf_config["surf_directions"]);
    int num_of_surf_object = surf_directions.cols();
    surf_directions = rotation_transform * surf_directions;
	ConcatMatrix(surf_directions_, surf_directions);
    ConcatMatrix(opt_surf_directions_, surf_directions);
    
    // surf centers
	Eigen::MatrixXd surf_centeral_points = MatrixParseT(surf_config["surf_centers"]);
    surf_centeral_points = rotation_transform * surf_centeral_points + 
        transition_transform.replicate(1, num_of_surf_object);
	ConcatMatrix(surf_cenetral_points_, surf_centeral_points);
    ConcatMatrix(opt_surf_cenetral_points_, surf_centeral_points);

    // surf boundary directions
	Eigen::MatrixXd surf_boundary_directions = MatrixParseT(surf_config["surf_boundary_directions"]);
    surf_boundary_directions = rotation_transform * surf_boundary_directions;
	ConcatMatrix(surf_boundary_directions_, surf_boundary_directions);

    // surf boundary points
	Eigen::MatrixXd surf_boundary_points = MatrixParseT(surf_config["surf_boundary_points"]);
	surf_boundary_points = rotation_transform * surf_boundary_points
         + transition_transform.replicate(1, 2 * num_of_surf_object);
    ConcatMatrix(surf_boundary_points_, surf_boundary_points);

	Eigen::MatrixXd surf_radius = VetorParse(surf_config["surf_radius"]);
    Eigen::MatrixXd surf_radius_t = surf_radius.transpose();
	ConcatMatrix(surf_radius_, surf_radius_t);

	surf_feature_index_object_[current_object_id_].clear();
	for (unsigned int index = num_of_surf_; index < (num_of_surf_ + num_of_surf_object); index++) {
		// assert
		assert(feature_deprecated_["surf"].size() == index);
		feature_deprecated_["surf"].push_back(false);
		surf_feature2id.push_back(current_object_id_);
		surf_feature_index_object_[current_object_id_].push_back(index);
	}

	num_of_surf_ += num_of_surf_object;

	for (auto v : surf_config["surf_approximated"].as_array()) {
		feature_approximated_["surf"].push_back((bool)v);
	}
};

/*
What "remove object" is doing here is that it deprecates the object and its related features
Lazy delete
*/
void SceneInference::RemoveObject(int object_id) {
	object_deprecated_[object_id] = true;

	// deprecate plane & surf feature
	for (auto index : plane_feature_index_object_[object_id]) {
		feature_deprecated_["plane"][index] = true;
	}

	for (auto index : surf_feature_index_object_[object_id]) {
		feature_deprecated_["surf"][index] = true;
	}
		
};

/*
Update object pose
The update should be a relative transformation
*/
void SceneInference::UpdateObjectPose(int object_id, const Eigen::Ref<Eigen::MatrixXd>& object_pose) {
	Eigen::MatrixXd relative_pose = object_pose * (object_poses_[object_id].inverse());
    object_poses_[object_id] = object_pose;

    if (object_deprecated_[object_id]) {
		return;
	}
	else {
		Eigen::MatrixXd rotation_matrix = relative_pose.block(0, 0, 3, 3);
		Eigen::MatrixXd transition_matrix = relative_pose.block(0, 3, 3, 1);
		
        // plane
		for (auto index : plane_feature_index_object_[object_id]) {
			plane_normals_.block(0, index, 3, 1) =
				rotation_matrix * plane_normals_.block(0, index, 3, 1);
			plane_central_points_.block(0, index, 3, 1) =
				rotation_matrix * plane_central_points_.block(0, index, 3, 1) + transition_matrix;
			plane_boundary_points_.block(0, 4 * index, 3, 4) = rotation_matrix *
				plane_boundary_points_.block(0, 4 * index, 3, 4) + transition_matrix.replicate(1, 4);
		}

        // surf
		for (auto index : surf_feature_index_object_[object_id]) {
			surf_directions_.block(0, index, 3, 1) =
				rotation_matrix * surf_directions_.block(0, index, 3, 1);
			surf_boundary_directions_.block(0, 2 * index, 3, 2) =
				rotation_matrix * surf_boundary_directions_.block(0, 2 * index, 3, 2);
			
			surf_cenetral_points_.block(0, index, 3, 1) = rotation_matrix *
				surf_cenetral_points_.block(0, index, 3, 1) + transition_matrix;
			surf_boundary_points_.block(0, 2 * index, 3, 2) = rotation_matrix *
				surf_boundary_points_.block(0, 2 * index, 3, 2) + transition_matrix.replicate(1, 2);
		}

        // gravity center
        object_gravity_centers_[object_id] = rotation_matrix * object_gravity_centers_[object_id]
            + transition_matrix;
	}
};

/*
Currently, we naively think that the table pose is the gravity pose
 */
void SceneInference::SetGravity(const Eigen::Ref<Eigen::MatrixXd>& gravity_pose){
    // TODO: How to detect gravity pose & gravity direction
    // TODO: Get gravity from pose_estimation
    gravity_pose_ = gravity_pose;
    Eigen::Vector4d global_z_axis(0, 0, -1, 0);
    gravity_direction_ = (gravity_pose_ * global_z_axis).head(3);
};

/* 
Calculate Difference between two different feature
*/
void SceneInference::CalculateDiff() {
	// Initilize Eigen matrix before calculation
    angle_diff_plane2plane_ = Eigen::MatrixXd::Zero(num_of_plane_, num_of_plane_);
    dist_diff_plane2plane_ = Eigen::MatrixXd::Zero(num_of_plane_, num_of_plane_);
    angle_diff_plane2surf_ = Eigen::MatrixXd::Zero(num_of_plane_, num_of_surf_);
    dist_diff_plane2surf_ = Eigen::MatrixXd::Zero(num_of_plane_, num_of_surf_);
    dist_diff_surf2surf_ = Eigen::MatrixXd::Zero(num_of_surf_, num_of_surf_);
    plane_feature_supporting_ = Eigen::MatrixXd::Zero(1, num_of_plane_);
    surf_feature_supporting_ = Eigen::MatrixXd::Zero(1, num_of_surf_);

    // Plane2Plane
    if(num_of_plane_ != 0)
        CalculateDiffPlane2Plane(plane_normals_, plane_central_points_,
            angle_diff_plane2plane_, dist_diff_plane2plane_);

    // DisplayMatrix(dist_diff_plane2plane_.block(0, 0, 1, num_of_plane_), "Dist Plane2Plane");
    // DisplayMatrix(angle_diff_plane2plane_.block(0, 0, 1, num_of_plane_), "Angle Plane2Plane");

	// Plane2Surf
    if((num_of_plane_ != 0) && (num_of_surf_ != 0) )
        CalculateDiffPlane2Surf(plane_normals_, surf_directions_, plane_central_points_,
            surf_cenetral_points_, angle_diff_plane2surf_, dist_diff_plane2surf_);
        
    /*
    // TODO: delete it
    DisplayMatrix(dist_diff_plane2surf_.block(7, 0, 6, num_of_surf_), "Dist Plane2Surf");
    DisplayMatrix(angle_diff_plane2surf_.block(7, 0, 6, num_of_surf_), "Angle Plane2Surf");
     */
    

	// Surf2Surf
    if(num_of_surf_ != 0)
        CalculateDiffSurf2Surf(surf_directions_, surf_cenetral_points_, dist_diff_surf2surf_);

	// Calculate the Supportting Result
    if(num_of_plane_ != 0)
	    SupportingStatus(plane_normals_, gravity_direction_, plane_feature_supporting_);
    if(num_of_surf_ != 0)
	    SupportingStatusV2(surf_directions_, gravity_direction_, surf_feature_supporting_);

};

/*
Feature Supporting Relationship
Infer the supporting relationship between features : Supporting & Supported
Find the relationship between all features
*/
void SceneInference::FeatureSupportingRelation() {
	feature_supporting_ = Eigen::MatrixXd::Zero((num_of_plane_ + num_of_surf_),
		(num_of_plane_ + num_of_surf_));

	// Plane2Plane : i & j are global feature index
	for (int i = 0; i < num_of_plane_; i++) {
		for (int j = i + 1; j < num_of_plane_; j++) {
			// first judge if we need to do this check
			bool no_contact = false;
			if (feature_deprecated_["plane"][i] || feature_deprecated_["plane"][j]) {
				no_contact = true;
			}

			if (plane_feature2id[i] == plane_feature2id[j]) {
				// two feature belong to same object id
				no_contact = true;
			}

			if (no_contact) {
				continue;
			}

			// If one feature is approximate, threshold will be leverage
			double dist_threshold = plane2plane_dist_threshold;
			double angular_threshold = plane2plane_angular_threshold;
			if (feature_approximated_["plane"][i] || feature_approximated_["plane"][j]) {
				dist_threshold *= unprecise_leverage;
				angular_threshold *= unprecise_leverage;
			}

			if (dist_diff_plane2plane_(i, j) < dist_threshold) {
				if (angle_diff_plane2plane_(i, j) < angular_threshold) {
					// If distance obey threshold, exam space consistency
					bool spatial_consistency =
						SpatialMatchingCheck(plane_boundary_points_.block(0, 4 * i, 3, 4),
							plane_boundary_points_.block(0, 4 * j, 3, 4));
					if (spatial_consistency) {
						if (plane_feature_supporting_(i) == 1) {
							// i supported by j
							assert(plane_feature_supporting_(j) == -1);
							feature_supporting_(i, j) = 1.0;
							feature_supporting_(j, i) = -1.0;
						}
						else if (plane_feature_supporting_(j) == 1) {
							// j supported by i
							assert(plane_feature_supporting_(i) == -1);
							feature_supporting_(j, i) = 1.0;
							feature_supporting_(i, j) = -1.0;
						}
					}
				}
			}
		}
	}

	// Plane2Surf : Surface Contact
	for (int i = 0; i < num_of_plane_; i++) {
		for (int j = 0; j < num_of_surf_; j++) {
			// first judge if we need to do this check
			bool no_contact = false;
			if (feature_deprecated_["plane"][i] || feature_deprecated_["surf"][j]) {
				no_contact = true;
			}

			if (plane_feature2id[i] == surf_feature2id[j]) {
				// two feature belong to same object id
				no_contact = true;
			}

			if (no_contact) {
				continue;
			}

			// If one feature is approximate, threshold will be leverage
			double dist_threshold = plane2surf_dist_threshold;
			double angular_threshold = plane2surf_angular_threshold;
			if (feature_approximated_["plane"][i] || feature_approximated_["surf"][j]) {
				dist_threshold *= unprecise_leverage;
				angular_threshold *= unprecise_leverage;
			}

			if (abs(dist_diff_plane2surf_(i, j) - surf_radius_(j)) < dist_threshold) {
				if (angle_diff_plane2surf_(i, j) < angular_threshold) {
					// If distance obey threshold, exam space consistency
                    Eigen::Vector3d this_plane_normal = plane_normals_.block(0, i, 3, 1);
                    Eigen::Vector3d this_surf_direction = surf_directions_.block(0, j, 3, 1);
					Eigen::Vector3d this_start_direction = surf_boundary_directions_.block(0, 2 * j, 3, 1);
                    Eigen::Vector3d this_end_direction = surf_boundary_directions_.block(0, 2 * j + 1, 3, 1);
                    
                    bool spatial_consistency = SpatialMatchingCheckV3(
                                                this_plane_normal,
                                                this_surf_direction,
                                                this_start_direction,
                                                this_end_direction,
                                                plane_boundary_points_.block(0, 4 * i, 3, 4),
                                                surf_boundary_points_.block(0, 2 * j, 3, 2));

					if (spatial_consistency) {
						if (plane_feature_supporting_(i) == 1 &&
							surf_feature_supporting_(j) == 1) {
							// i supported j
							feature_supporting_(i, num_of_plane_ + j) = 1.0;
							feature_supporting_(num_of_plane_ + j, i) = -1.0;
						}
						else if (plane_feature_supporting_(i) == -1 &&
							surf_feature_supporting_(j) == 1) {

							// j supported i
							feature_supporting_(num_of_plane_ + j, i) = 1.0;
							feature_supporting_(i, num_of_plane_ + j) = -1.0;
						}
					}
				}
			}

		}
	}

	// Surf2Surf
	for (int i = 0; i < num_of_surf_; i++) {
		for (int j = i + 1; j < num_of_surf_; j++) {
			// first judge if we need to do this check
			bool no_contact = false;
			if (feature_deprecated_["surf"][i] || feature_deprecated_["surf"][j]) {
				no_contact = true;
			}

			if (surf_feature2id[i] == surf_feature2id[j]) {
				// two feature belong to same object id
				no_contact = true;
			}

			if (no_contact) {
				continue;
			}

            // we make a limitation over surf2surf, only large surf can support each other
            if((surf_radius_(i) < minimum_surf_threshold) || (surf_radius_(j) < minimum_surf_threshold)){
                continue;
            }

			// If one feature is approximate, threshold will be leverage
			double dist_threshold = surf2surf_dist_threshold;

			if (feature_approximated_["surf"][i] || feature_approximated_["surf"][j]) {
				dist_threshold *= unprecise_leverage;
			}

			if (abs(dist_diff_surf2surf_(i, j) - (surf_radius_(i) + surf_radius_(j)))
				< dist_threshold) {
				// Maybe in the future, I can do more detailed analysis
				bool spatial_consistency = SpatialMatchingCheckV2(
					surf_boundary_points_.block(0, 2 * i, 3, 2),
					surf_boundary_points_.block(0, 2 * j, 3, 2));

				if (spatial_consistency) {
					if (surf_feature_supporting_(i) == 1 && surf_feature_supporting_(j) == 1) {
						Eigen::Vector3d this_edge = surf_directions_.block(0, i, 3, 1);
						Eigen::Vector3d that_edge = surf_directions_.block(0, j, 3, 1);

						Eigen::Vector3d inner_normal = this_edge.cross(that_edge);

						Eigen::Vector3d vector_j2i = surf_cenetral_points_.block(0, i, 3, 1) -
							surf_cenetral_points_.block(0, j, 3, 1);

						double normal_direction = inner_normal.dot(gravity_direction_);
						double direction_j2i = normal_direction * (vector_j2i.dot(inner_normal));
						if (direction_j2i > 0) {
							// j upper, i lower : j supported by i
							feature_supporting_(j + num_of_plane_, i + num_of_plane_) = 1.0;
							feature_supporting_(i + num_of_plane_, j + num_of_plane_) = -1.0;
						}
						else {
							// i supported j
							feature_supporting_(j + num_of_plane_, i + num_of_plane_) = -1.0;
							feature_supporting_(i + num_of_plane_, j + num_of_plane_) = 1.0;
						}
					}
				}
			}
		}
	}

    // DisplayMatrix(feature_supporting_, "Feature Supporting");
};

bool SceneInference::ObjectSupportStatus(int object_id, 
	                                     std::map<int, std::pair<int, int> >& id2feature) {
	// Exam plane first
	// Construct a feature manager to manage the feature 
	ProjectionFeatureManager project_feature_manager(object_gravity_centers_[object_id],
													 gravity_pose_);

	/*
	Feature here should be added in some specific order:
	From Plane2Plane to Plane2Surf to Surf2Surf
	*/
	id2feature.clear();
	bool supporting_status = false;

	int feature_count = 0;
	for (unsigned int i = 0; i < plane_feature_index_object_[object_id].size(); i++) {
		// take out a plane, check all supporting feature
		int plane_index = plane_feature_index_object_[object_id][i];
		for (int j = 0; j < num_of_plane_; j++) {
			if (feature_supporting_(plane_index, j) == 1) {
				// a supported feature, add it to feature manger, check gravity
				// i is the other plane
				project_feature_manager.AddFeature(feature_count,
					plane_boundary_points_.block(0, 4 * plane_index, 3, 4),
					plane_boundary_points_.block(0, 4 * j, 3, 4));
				
				// log current feature
				id2feature[feature_count] = std::pair<int, int>(plane_index, j);
				feature_count++;

				// after adding the new feature & check supporting status
				if (project_feature_manager.SupportingStatus()) {
					supporting_status = true;
					// stop loop in advance
					i = plane_feature_index_object_[object_id].size();
					j = num_of_plane_;
                    break;
				}
			}
		}
	}

	// check plane2surf feature
	for (unsigned int i = 0; i < plane_feature_index_object_[object_id].size(); i++) {
		int plane_index = plane_feature_index_object_[object_id][i];
		for (int j = num_of_plane_; j < num_of_plane_ + num_of_surf_; j++) {
			// j is the surfs
			if (supporting_status) {
				// if fully supported, then directy quit loop
				i = plane_feature_index_object_[object_id].size();
				j = num_of_plane_ + num_of_surf_;
                break;
			}

			if (feature_supporting_(plane_index, j) == 1) {
				// a supported surf2plane
				project_feature_manager.AddFeature(feature_count,
					plane_boundary_points_.block(0, 4 * plane_index, 3, 4),
					surf_boundary_points_.block(0, 2 * (j - num_of_plane_), 3, 2));

				// log current feature
				id2feature[feature_count] = std::pair<int, int>(plane_index, j);
				feature_count++;

				// after adding the new feature & check supporting status 
				if (project_feature_manager.SupportingStatus()) {
					supporting_status = true;
				}
			}
		}
	}

	// check surf2plane feature
	// check plane2surf feature
	for (unsigned int i = 0; i < surf_feature_index_object_[object_id].size(); i++) {
		int surf_index = surf_feature_index_object_[object_id][i];
		for (int j = 0; j < num_of_plane_ ; j++) {
			// j is the surfs
			if (supporting_status) {
				// if fully supported, then directy quit loop
				i = surf_feature_index_object_[object_id].size();
				j = num_of_plane_;
                break;
			}

			if (feature_supporting_(num_of_plane_ + surf_index, j) == 1) {
				// a supported surf2plane
				project_feature_manager.AddFeature(feature_count,
					surf_boundary_points_.block(0, 2 * surf_index, 3, 2),
					plane_boundary_points_.block(0, 4 * j, 3, 4));

				// log current feature
				id2feature[feature_count] = std::pair<int, int>(num_of_plane_ + surf_index, j);
				feature_count++;

				// after adding the new feature & check supporting status 
				if (project_feature_manager.SupportingStatus()) {
					supporting_status = true;
				}
			}
		}
	}

	// check surf2surf
	for (unsigned int i = 0; i < surf_feature_index_object_[object_id].size(); i++) {
		int surf_index = surf_feature_index_object_[object_id][i];
		for (int j = num_of_plane_; j < num_of_plane_ + num_of_surf_; j++) {
			// j is the surfs
			if (supporting_status) {
				// if fully supported, then directy quit loop
				i = surf_feature_index_object_[object_id].size();
				j = num_of_plane_ + num_of_surf_;
                break;
			}

			if (feature_supporting_(num_of_plane_ + surf_index, j) == 1) {
				// a supported surf2plane
				project_feature_manager.AddFeature(feature_count,
					surf_boundary_points_.block(0, 2 * surf_index, 3, 2),
					surf_boundary_points_.block(0, 2* (j - num_of_plane_), 3, 2));

				// log current feature
				id2feature[feature_count] = std::pair<int, int>(num_of_plane_ + surf_index, j);
				feature_count++;

				// after adding the new feature & check supporting status 
				if (project_feature_manager.SupportingStatus()) {
					supporting_status = true;
				}
			}
		}
	}

	// after all check feature supporting has been checked
	if (!supporting_status) {
		// this object is not fully suppported
		return false;
	}

	// other case find the smallest supporting case by remove unessary part
	for (int i = feature_count - 2; i >= 0; i--) {
		// traverse inversely to remove unessary feature, 
		// the last feature is always needed
		project_feature_manager.RemoveFeature(i);
		if (!project_feature_manager.SupportingStatus()) {
			// if not valid after remove, leave this
			project_feature_manager.RecoverLastRemove();
		}
		else {
			// this feature is not needed
			id2feature.erase(i); 
		}
	}

	return true;
};

void SceneInference::DisplayRelationship(){
    for(auto object_relationship : object_relationship_){
        int object_id = object_relationship.first;
        if(object_deprecated_[object_id]){
            continue;
        }
        std::cout << " --------------------- " << std::endl;
        std::cout << "Object : " << object_names_[object_id] << " : "
                  << object_id << std::endl;

        for(auto feature : object_relationship.second) {
            if(feature.first >= num_of_plane_){
                std::cout << "surf " << feature.first << " of ";
                int feature_object_id = surf_feature2id[feature.first - num_of_plane_];
                std::cout << object_names_[feature_object_id] << ", ";
            }
            else{
                std::cout << "plane " << feature.first << " of ";
                int feature_object_id = plane_feature2id[feature.first];
                std::cout << object_names_[feature_object_id] << ", ";
            }

            std::cout << " | ";

            if(feature.second >= num_of_plane_){
                std::cout << "surf " << feature.second << " of ";
                int feature_object_id = surf_feature2id[feature.second - num_of_plane_];
                std::cout << object_names_[feature_object_id] << ".";
            }
            else{
                std::cout << "plane " << feature.second << " of ";
                int feature_object_id = plane_feature2id[feature.second];
                std::cout << object_names_[feature_object_id] << ".";
            }

            std::cout << std::endl;
        }
    }
};

void SceneInference::DisplayRelationship(std::map<int, std::vector<int> >& clusters_relationship){
	clusters_relationship.clear(); // clear previous relationships for clusters
    for(auto object_relationship : object_relationship_){
        int object_id = object_relationship.first;
        if(object_deprecated_[object_id]){
            continue;
        }
        std::cout << " --------------------- " << std::endl;
        std::cout << "Object : " << object_names_[object_id] << " : "
                  << object_id << std::endl;

        for(auto feature : object_relationship.second) {
            if(feature.first >= num_of_plane_){
                std::cout << "surf " << feature.first << " of ";
                int feature_object_id = surf_feature2id[feature.first - num_of_plane_];
                std::cout << object_names_[feature_object_id] << ", ";
            }
            else{
                std::cout << "plane " << feature.first << " of ";
                int feature_object_id = plane_feature2id[feature.first];
                std::cout << object_names_[feature_object_id] << ", ";
            }

            std::cout << " | ";

            if(feature.second >= num_of_plane_){
                std::cout << "surf " << feature.second << " of ";
                int feature_object_id = surf_feature2id[feature.second - num_of_plane_];
                std::cout << object_names_[feature_object_id] << ".";
				clusters_relationship[object_id].push_back(feature_object_id);
				clusters_relationship[feature_object_id].push_back(object_id);
            }
            else{
                std::cout << "plane " << feature.second << " of ";
                int feature_object_id = plane_feature2id[feature.second];
                std::cout << object_names_[feature_object_id] << ".";
				clusters_relationship[object_id].push_back(feature_object_id);
				clusters_relationship[feature_object_id].push_back(object_id);
            }

            std::cout << std::endl;
        }
    }
};

void SceneInference::LogSceneStatus(std::string log_file_name){
    configuru::Config cfg = configuru::Config::object();
    // update object level information 
    // object_name_object_id
    for(auto object : object_relationship_){
        int object_id = object.first;
        if(object_deprecated_[object_id]){
            continue;
        }

        std::string object_name = object_names_[object_id];
        std::string object_unique_name = object_name
            + "_" + std::to_string(object_id);
        
        std::string object_config_file = current_working_path_ + "/models/" + 
                                (std::string) object_register_config_[object_name];

        configuru::Config object_config = configuru::parse_file(object_config_file,
            configuru::JSON);
        
        // Add current object pose & relationship
        WriteMatrix4d(object_config, object_poses_[object_id], "object_pose");
        // Add relationship
        configuru::Config object_features = configuru::Config::array();
        for(auto feature : object.second){
            object_features.push_back(configuru::Config::array({
                feature.first, feature.second
            }));
        }
        object_config["object_feature_relation"] = object_features;
        cfg[object_unique_name] = object_config;
    }

    std::string current_path = GetCurrentWorkingDir();
	configuru::dump_file(current_path + 
                         log_file_name, cfg, configuru::JSON);
    
};

/*
Relationship Inferenece is a combined operation on geometry inference
 */
void SceneInference::RelationshipInference(bool output) {
	object_relationship_.clear();
	for (auto object : object_deprecated_) {
		if (!object.second) {
			// if object is not deprecated, then check its relationship
			std::map<int, std::pair<int, int> > relationship_object;
			bool supported_status = ObjectSupportStatus(object.first, relationship_object);
            if(output) {
                std::cout << "Supported Status of " << object_names_[object.first] << " : ";
                std::cout << object.first << " is " << supported_status << std::endl;
            }
             
            object_supported_[object.first] = supported_status;
			std::vector<std::pair<int, int> > relationships;
			object_relationship_[object.first] = relationships;
			for (auto supports : relationship_object) {
				object_relationship_[object.first].push_back(supports.second);
			}
		}
	}
};

void SceneInference::RelationshipInference(std::string log_file_name, 
                                           std::map<int, std::vector<int> >& clusters_relationship_,
                                           bool output){
    if(num_of_object_ == 0) {
        return;
    }
    CalculateDiff();
    FeatureSupportingRelation();
    RelationshipInference(output);
    DisplayRelationship(clusters_relationship_);
    LogSceneStatus(log_file_name);
};

void SceneInference::RelationshipInference(std::string log_file_name, bool output){
    if(num_of_object_ == 0) {
        return;
    }
	CalculateDiff();
    FeatureSupportingRelation();
    RelationshipInference(output);
    if(output)
        DisplayRelationship();
    LogSceneStatus(log_file_name);
};

void SceneInference::FeatureForOptimization(std::vector<Eigen::MatrixXd>& normal_1s,
                                std::vector<Eigen::MatrixXd>& normal_2s,
                                std::vector<Eigen::MatrixXd>& center_1s,
                                std::vector<Eigen::MatrixXd>& center_2s,
                                std::vector<double>& radius_1s,
                                std::vector<double>& radius_2s,
                                std::vector<int>& object_id_1s, 
                                std::vector<int>& object_id_2s,
                                std::vector<int>& support_types) {
    
    for(auto object_relationship : object_relationship_){
        int object_id = object_relationship.first;
        // if object deprecated
        if(object_deprecated_[object_id]){
            continue;
        } 
        // if object not fully supported
        if(!object_supported_.at(object_id)) {
            continue;
        }

        for(auto feature : object_relationship.second) {
            int support_flag = 0;
            if(feature.first >= num_of_plane_){
                // surf
                support_flag += 0;
                int feature_object_id = surf_feature2id[feature.first - num_of_plane_];
                object_id_1s.push_back(feature_object_id);
                // normal & center
                normal_1s.push_back(opt_surf_directions_.block(0, feature.first - num_of_plane_, 3, 1));
                center_1s.push_back(opt_surf_cenetral_points_.block(0, feature.first - num_of_plane_, 3, 1));
                radius_1s.push_back(surf_radius_(feature.first - num_of_plane_));
            }
            else{
                // plane
                support_flag += 10;
                int feature_object_id = plane_feature2id[feature.first];
                object_id_1s.push_back(feature_object_id);
                // normal & center
                normal_1s.push_back(opt_plane_normals_.block(0, feature.first, 3, 1));
                center_1s.push_back(opt_plane_central_points_.block(0, feature.first, 3, 1));
                radius_1s.push_back(0);
            }

            if(feature.second >= num_of_plane_){
                // surf
                support_flag += 0;
                int feature_object_id = surf_feature2id[feature.second - num_of_plane_];
                object_id_2s.push_back(feature_object_id);
                // normal & center
                normal_2s.push_back(opt_surf_directions_.block(0, feature.second - num_of_plane_, 3, 1));
                center_2s.push_back(opt_surf_cenetral_points_.block(0, feature.second - num_of_plane_, 3, 1));
                radius_2s.push_back(surf_radius_(feature.second - num_of_plane_));
            }
            else{
                // plane
                support_flag += 1;
                int feature_object_id = plane_feature2id[feature.second];
                object_id_2s.push_back(feature_object_id);
                // normal & center
                normal_2s.push_back(opt_plane_normals_.block(0, feature.second, 3, 1));
                center_2s.push_back(opt_plane_central_points_.block(0, feature.second, 3, 1));
                radius_2s.push_back(0);
            }

            support_types.push_back(support_flag);
        }
    }                             
};

void SceneInference::FeatureForOptimization(std::vector<Eigen::MatrixXd>& transform_1s,
                            std::vector<Eigen::MatrixXd>& transform_2s,
                            std::vector<double>& transform_distances,
                            std::vector<int>& object_id_1s,
                            std::vector<int>& object_id_2s,
                            std::vector<int>& support_types) {
     for(auto object_relationship : object_relationship_){
        int object_id = object_relationship.first;
        if(object_deprecated_[object_id]){
            continue;
        } 

        for(auto feature : object_relationship.second) {
            int support_flag = 0;
            // z axis is the direction we choose
            double transform_distance = 0;
            Eigen::Vector3d z_axis;
            z_axis << 0, 0, 1;

            // object 1:
            if(feature.first >= num_of_plane_){
                // surf
                support_flag += 0;
                int feature_object_id = surf_feature2id[feature.first - num_of_plane_];
                object_id_1s.emplace_back(feature_object_id);
                // normal
                Eigen::Vector3d normal_1s = opt_surf_directions_.block(0, feature.first - num_of_plane_, 3, 1);

                // rotation axis
                Eigen::Vector3d rotation_axis = z_axis.cross(normal_1s);
                // rotation angle
                double dot_result = z_axis.dot(normal_1s);
                double rotation_angle = std::acos(dot_result);
                // Roderegas
                Eigen::MatrixXd relative_transform(4, 4);
                Rogas(rotation_axis, rotation_angle, relative_transform.block(0, 0, 3, 3));
                relative_transform.block(0, 3, 3, 1) = 
                    opt_surf_cenetral_points_.block(0, feature.first - num_of_plane_, 3, 1);

                transform_distance += surf_radius_(feature.first - num_of_plane_);  // plane need to add surf radius
                transform_1s.emplace_back(relative_transform);

            }
            else{
                // plane
                support_flag += 10;
                int feature_object_id = plane_feature2id[feature.first];
                object_id_1s.emplace_back(feature_object_id);
                // normal & center
                Eigen::Vector3d normal_1s = opt_plane_normals_.block(0, feature.first, 3, 1);
                // rotation axis
                Eigen::Vector3d rotation_axis = z_axis.cross(normal_1s);
                // rotation angle
                double dot_result = z_axis.dot(normal_1s);
                double rotation_angle = std::acos(dot_result);
                // Roderegas
                Eigen::MatrixXd relative_transform(4, 4);
                Rogas(rotation_axis, rotation_angle, relative_transform.block(0, 0, 3, 3));

                relative_transform.block(0, 3, 3, 1) = opt_plane_central_points_.block(0, feature.first, 3, 1);
                transform_distance += 0;  // plane need to add surf radius
                transform_1s.emplace_back(relative_transform);
            }

            if(feature.second >= num_of_plane_){
                // surf
                support_flag += 0;
                int feature_object_id = surf_feature2id[feature.second - num_of_plane_];
                object_id_2s.emplace_back(feature_object_id);
                // normal
                Eigen::Vector3d normal_2s = opt_surf_directions_.block(0, feature.second - num_of_plane_, 3, 1);

                // rotation axis
                Eigen::Vector3d rotation_axis = z_axis.cross(normal_2s);
                // rotation angle
                double dot_result = z_axis.dot(normal_2s);
                double rotation_angle = std::acos(dot_result);
                // Roderegas
                Eigen::MatrixXd relative_transform(4, 4);
                Rogas(rotation_axis, rotation_angle, relative_transform.block(0, 0, 3, 3));
                relative_transform.block(0, 3, 3, 1) = 
                    opt_surf_cenetral_points_.block(0, feature.second - num_of_plane_, 3, 1);

                transform_distance += surf_radius_(feature.second - num_of_plane_);  // plane need to add surf radius
                transform_2s.emplace_back(relative_transform);
            }
            else{
                // plane
                support_flag += 1;
                int feature_object_id = plane_feature2id[feature.second];
                object_id_2s.emplace_back(feature_object_id);
                // normal & center
                Eigen::Vector3d normal_2s = opt_plane_normals_.block(0, feature.second, 3, 1);
                // rotation axis
                Eigen::Vector3d rotation_axis = z_axis.cross(normal_2s);
                // rotation angle
                double dot_result = z_axis.dot(normal_2s);
                double rotation_angle = std::acos(dot_result);
                // Roderegas
                Eigen::MatrixXd relative_transform(4, 4);
                Rogas(rotation_axis, rotation_angle, relative_transform.block(0, 0, 3, 3));

                relative_transform.block(0, 3, 3, 1) = opt_plane_central_points_.block(0, feature.second, 3, 1);
                transform_distance += 0;  // plane need to add surf radius
                transform_2s.emplace_back(relative_transform);
            }

            support_types.emplace_back(support_flag);
            transform_distances.emplace_back(transform_distance);
        }
    }
};
