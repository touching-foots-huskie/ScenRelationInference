#include "scene_inference.h"

//method 
void ProjectionFeatureManager::AddFeature(int feature_id,
                    const Eigen::Ref<Eigen::MatrixXd>& feature_boundary_points_1,
		            const Eigen::Ref<Eigen::MatrixXd>& feature_boundary_points_2) {
	
    gravity_center_ = rotation_transform_ * gravity_center_ + transition_transform_;

	// pairs 1
	Eigen::MatrixXd projected_boundary_points_1 = rotation_transform_ * feature_boundary_points_1
		+ transition_transform_;
	int N1 = projected_boundary_points_1.cols();
	projected_boundary_points_1.block(2, 0, 1, N1) *= 0;  // Remove z axis

	// pairs 2
	Eigen::MatrixXd projected_boundary_points_2 = rotation_transform_ * feature_boundary_points_2
		+ transition_transform_;
	int N2 = projected_boundary_points_2.cols();
	projected_boundary_points_2.block(2, 0, 1, N2) *= 0;  // Remove z axis

	// add index into feature index
	points_index_feature_[feature_id].push_back(boundary_points_.cols());
	points_index_feature_[feature_id].push_back(N1 + N2);

	norms_index_feature_[feature_id].push_back(projection_norms_.cols());
	norms_index_feature_[feature_id].push_back(N1 + N2);

	// add the proected data into projected boundary points & edges
	ConcatMatrix(boundary_points_, projected_boundary_points_1);
	ConcatMatrix(boundary_points_, projected_boundary_points_2);

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
	int points_start_index = points_index_feature_[feature_id][0];
	int points_feature_num = points_index_feature_[feature_id][1];

	int norms_start_index = norms_index_feature_[feature_id][0];
	int norms_feature_num = norms_index_feature_[feature_id][1];

	// make a copy first
	copy_boundary_points_ = boundary_points_;
	copy_projection_norms_ = projection_norms_;

	// Remove then
	RemoveMatrix(boundary_points_, points_start_index, points_feature_num);
	RemoveMatrix(projection_norms_, norms_start_index, norms_feature_num);

	points_index_feature_.erase(feature_id);
	norms_index_feature_.erase(feature_id);
};

void ProjectionFeatureManager::RecoverLastRemove() {
	boundary_points_ = copy_boundary_points_;
	projection_norms_ = copy_projection_norms_;
};

bool ProjectionFeatureManager::SupportingStatus() {
	// exam if current points & normals can support gravity center
	bool supported = GravitySupportCheck(boundary_points_,
		gravity_center_, projection_norms_);
	return supported;
};

SceneInference::SceneInference(std::string object_register_file)
{
	num_of_object_ = 0;
	num_of_plane_ = 0;
	num_of_surf_ = 0;

	object_register_file_ = object_register_file;
	object_register_config_ = configuru::parse_file(object_register_file_, configuru::JSON);
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

	std::string object_config_file = (std::string) object_register_config_[object_name];
	configuru::Config geometry_config = configuru::parse_file(object_config_file,
		configuru::JSON);

	object_name = (std::string) geometry_config["object_name"];
    // 
	Eigen::MatrixXd  gravity_center = VetorParse(geometry_config["gravity_center"]);
	object_gravity_centers_[current_object_id_] = gravity_center;

    // Inner Transform
    object_inner_transfrom_[object_id] = MatrixParseT(geometry_config["inner_transfrom"]);

	// Plane
	configuru::Config plane_config = geometry_config["plane_feature"];
	AddPlaneFeature(plane_config);

	// CySurf
	configuru::Config surf_config = geometry_config["cysurf_feature"];
	AddSurfFeature(surf_config);

	// initialize object pose
    object_poses_[object_id] = Eigen::MatrixXd::Identity(4, 4);
};

void SceneInference::AddPlaneFeature(configuru::Config& plane_config) {
	Eigen::MatrixXd plane_normal_object = MatrixParseT(plane_config["plane_normals"]);
	ConcatMatrix(plane_normals_, plane_normal_object);

	Eigen::MatrixXd plane_center_object = MatrixParseT(plane_config["plane_centers"]);
	ConcatMatrix(plane_central_points_, plane_center_object);

	Eigen::MatrixXd plane_boundary_points = MatrixParseT(plane_config["plane_boundary_points"]);
	ConcatMatrix(plane_boundary_points_, plane_boundary_points);

	int num_of_plane_obejct = plane_normal_object.cols();
	plane_feature_index_object_[current_object_id_].clear();

	for (unsigned int index = num_of_plane_; index < (num_of_plane_ + num_of_plane_obejct); index++) {
		// assert
		assert(feature_deprecated_["plane"].size() == index);
		feature_deprecated_["plane"].push_back(false);
		plane_feature2id.push_back(current_object_id_);
		plane_feature_index_object_[current_object_id_].push_back(index);

	}
	num_of_plane_ += num_of_plane_obejct;

	for (auto v : plane_config["plane_approximated"].as_array()) {
		feature_approximated_["plane"].push_back((bool)v);
	}
};

void SceneInference::AddSurfFeature(configuru::Config& surf_config) {
	Eigen::MatrixXd surf_directions = MatrixParseT(surf_config["surf_directions"]);
	ConcatMatrix(surf_directions_, surf_directions);

	Eigen::MatrixXd surf_cenetral_points = MatrixParseT(surf_config["surf_centers"]);
	ConcatMatrix(surf_cenetral_points_, surf_cenetral_points);

	Eigen::MatrixXd surf_boundary_directions = MatrixParseT(surf_config["surf_boundary_directions"]);
	ConcatMatrix(surf_boundary_directions_, surf_boundary_directions);

	Eigen::MatrixXd surf_boundary_points = MatrixParseT(surf_config["surf_boundary_points"]);
	ConcatMatrix(surf_boundary_points_, surf_boundary_points);

	Eigen::MatrixXd surf_radius = VetorParse(surf_config["surf_radius"]);
	ConcatMatrix(surf_radius_, surf_radius);

	int num_of_surf_object = surf_directions.cols();

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
		Eigen::Matrix3d rotation_matrix = relative_pose.block(0, 0, 3, 3);
		Eigen::Matrix3d transition_matrix = relative_pose.block(0, 3, 3, 1);
		// plane
		for (auto index : plane_feature_index_object_[object_id]) {
			plane_normals_.block(0, index, 3, 1) =
				rotation_matrix * plane_normals_.block(0, index, 3, 1);
			plane_central_points_.block(0, index, 3, 1) =
				rotation_matrix * plane_central_points_.block(0, index, 3, 1) + transition_matrix;
			plane_boundary_points_.block(0, 4 * index, 3, 4) = rotation_matrix *
				plane_boundary_points_.block(0, 4 * index, 3, 4) + transition_matrix.replicate(1, 4);
		}

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
	}
};

/* 
Calculate Difference between two different feature
*/
void SceneInference::CalculateDiff() {
	// Plane2Plane
	CalculateDiffPlane2Plane(plane_normals_, plane_central_points_,
		angle_diff_plane2plane_, dist_diff_plane2plane_);

	// Plane2Surf
	CalculateDiffPlane2Surf(plane_normals_, surf_directions_, plane_central_points_,
		surf_cenetral_points_, angle_diff_plane2surf_, dist_diff_plane2surf_);
	
	// Surf2Surf
	CalculateDiffSurf2Surf(surf_cenetral_points_, surf_cenetral_points_, dist_diff_surf2surf_);

	// Calculate the Supportting Result
	SupportingStatus(plane_normals_, plane_feature_supporting_, gravity_direction_);
	SupportingStatusV2(surf_directions_, surf_feature_supporting_, gravity_direction_);

};

/*
Feature Supporting Relationship
Infer the supporting relationship between features : Supporting & Supported
Find the relationship between all features
*/
void SceneInference::FeatureSupportingRelation() {
	feature_supporting_ = Eigen::MatrixXd::Zero((num_of_plane_ + num_of_surf_),
		(num_of_plane_ + num_of_surf_));

	// Plane2Plane
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

	// Plane2Surf
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
				if (angle_diff_plane2plane_(i, j) < angular_threshold) {
					// If distance obey threshold, exam space consistency
                    Eigen::Vector3d this_surf_direction = surf_directions_.block(0, j, 3, 1);
					Eigen::Vector3d this_start_direction = surf_boundary_directions_.block(0, 2 * j + 1, 3, 1);
                    Eigen::Vector3d this_end_direction = surf_boundary_directions_.block(0, 2 * j + 1, 3, 1);
                    bool spatial_consistency = SpatialMatchingCheckV3(
                                                this_surf_direction,
                                                this_start_direction,
                                                this_end_direction,
                                                plane_boundary_points_.block(0, 4 * i, 3, 4),
                                                surf_boundary_points_.block(0, 2 * j, 3, 2));

					if (spatial_consistency) {
						if (plane_feature_supporting_(i) == 1 &&
							surf_feature_supporting_(j) == 1) {
							// i supported j
							feature_supporting_(i, j) = 1.0;
							feature_supporting_(j, i) = -1.0;
						}
						else if (plane_feature_supporting_(i) == -1 &&
							surf_feature_supporting_(j) == 1) {

							// j supported i
							feature_supporting_(j, i) = 1.0;
							feature_supporting_(i, j) = -1.0;
						}
					}
				}
			}

		}
	}

	// Surf2Surf
	for (int i = 0; i < num_of_surf_; i++) {
		for (int j = 0; j < num_of_surf_; j++) {
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

			// If one feature is approximate, threshold will be leverage
			double dist_threshold = surf2surf_dist_threshold;

			if (feature_approximated_["surf"][i] || feature_approximated_["surf"][j]) {
				dist_threshold *= unprecise_leverage;
			}

			if (abs(dist_diff_surf2surf_(i, j) - (surf_radius_(i) + surf_radius_(j)))
				< dist_threshold) {
				// Maybe in the future, I can do more detailed analysis
				bool spatial_consistency = SpatialMatchingCheckV2(
					surf_boundary_points_.block(0, 2 * i, 3, 1),
					surf_boundary_points_.block(0, 2 * j, 3, 1));
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
							feature_supporting_(j, i) = 1.0;
							feature_supporting_(i, j) = -1.0;
						}
						else {
							// i supported j
							feature_supporting_(j, i) = -1.0;
							feature_supporting_(i, j) = 1.0;
						}
					}
				}
			}
		}
	}
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

void SceneInference::RelationshipInference() {
	object_relationship_.clear();
	for (auto object : object_deprecated_) {
		if (!object.second) {
			// if object is not deprecated, then check its relationship
			std::map<int, std::pair<int, int> > relationship_object;
			ObjectSupportStatus(object.first, relationship_object);
			std::vector<std::pair<int, int> > relationships;
			object_relationship_[object.first] = relationships;
			for (auto supports : relationship_object) {
				object_relationship_[object.first].push_back(supports.second);
			}
		}
	}
};

void SceneInference::DisplayRelationship(){
    for(auto object_relationship : object_relationship_){
        int object_id = object_relationship.first;
        std::cout << " --------------------- " << std::endl;
        std::cout << "Object : " << object_names_[object_id] << " : "
                  << object_id << std::endl;

        for(auto feature : object_relationship.second){
            if(feature.first >= num_of_plane_){
                std::cout << "surf : " << feature.first;
            }
            else{
                std::cout << "plane : " << feature.first;
            }

            std::cout << " | ";

            if(feature.second >= num_of_plane_){
                std::cout << "surf : " << feature.second;
            }
            else{
                std::cout << "plane : " << feature.second;
            }

            std::cout << std::endl;
        }
    }
};
