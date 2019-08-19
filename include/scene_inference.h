#ifndef SCENE_INFERENCE
#define SCENE_INFERENCE

#include "geometry_feature.h"
#include "configuru.hpp"
#include "parse_tool.h"

// const value
const double plane2plane_angular_threshold = 0.02;
const double plane2plane_dist_threshold = 0.025;

const double plane2surf_angular_threshold = 0.15;  // different between plane2plane & plane2surf
const double plane2surf_dist_threshold = 0.025;

const double surf2surf_dist_threshold = 0.025;

// If one feature is unprecise, then we can leverage the threshold
const double unprecise_leverage = 3.0;

const double minimum_surf_threshold = 0.03;  // only surf larger than this can have supporting

/*
Control for one object
This class is used to control the process of managing feature suppporting
gravity_center is the position of gravity center in original coordinate
gravity_transform is the transform from ordinary coordinates to gravity coordinate
gravity_center|Dim:[D, 1]
gravity_transform|Dim:[4, 4]
In Feature Manager, Feature appears in pairs, the difference here is that there is
no edges between the two pairs of features
Input : gravity_center : MatrixXd
*/
class ProjectionFeatureManager {
public:
	ProjectionFeatureManager(const Eigen::Ref<Eigen::MatrixXd>& gravity_center,
                             const Eigen::Ref<Eigen::MatrixXd>& gravity_transform) {
		// gravity transformation coordinates
        Eigen::MatrixXd inverse_gravity_transform = gravity_transform.inverse();
        rotation_transform_ = inverse_gravity_transform.block(0, 0, 3, 3);
		transition_transform_ = inverse_gravity_transform.block(0, 3, 3, 1);
		// the gravity center should be the projected one
		gravity_center_ = rotation_transform_ * gravity_center + transition_transform_;
		gravity_center_.row(2) *= 0;  // clear z = 0, make the projection
	};

	// Two pairs of feature
	void AddFeature(int feature_id, 
                    const Eigen::Ref<Eigen::MatrixXd>& feature_boundary_points_1,
		            const Eigen::Ref<Eigen::MatrixXd>& feature_boundary_points_2);

	void RemoveFeature(int feature_id);
	void RecoverLastRemove();  // recover the last remove

	// judge if current features can provide a feasible supporting
	bool SupportingStatus();
	std::map<int, std::vector<int> > points_1_index_feature_; // points self
    std::map<int, std::vector<int> > points_2_index_feature_; // points outside
	std::map<int, std::vector<int> > norms_index_feature_;

	Eigen::MatrixXd boundary_points_1_;
    Eigen::MatrixXd boundary_points_2_;
	Eigen::MatrixXd projection_norms_;

	Eigen::MatrixXd copy_boundary_points_1_;
    Eigen::MatrixXd copy_boundary_points_2_;
	Eigen::MatrixXd copy_projection_norms_;

	Eigen::MatrixXd rotation_transform_;
	Eigen::MatrixXd transition_transform_;
	Eigen::MatrixXd gravity_center_;
};

class SceneInference {
public:
	SceneInference(std::string object_register_file);
	bool IsObjectIn(int object_id);
	void AddObject(int object_id, std::string object_name);

	// Manage Feature through different feature types
	void AddPlaneFeature(configuru::Config& plane_config);
	void AddSurfFeature(configuru::Config& plane_config);

	void RemoveObject(int object_id);  // Object Removal is using a lasy remove
	void UpdateObjectPose(int object_id, const Eigen::Ref<Eigen::MatrixXd>& object_pose);
    void SetGravity(const Eigen::Ref<Eigen::MatrixXd>& gravity_pose);

	void CalculateDiff(); 
	void FeatureSupportingRelation();
	bool ObjectSupportStatus(int object_id, std::map<int, std::pair<int, int> >& id2feature);
	
	void DisplayRelationship();
    void DisplayRelationship(std::map<int, std::vector<int> >& clusters_relationship);
    void LogSceneStatus(std::string log_file_name);
    
    void RelationshipInference(bool output);
    void RelationshipInference(std::string log_file_name,
                               std::map<int, std::vector<int> >& clusters_relationship,
                               bool output);
	void RelationshipInference(std::string log_file_name, bool output);
	
    // Version1
    void FeatureForOptimization(std::vector<Eigen::MatrixXd>& normal_1s,
                                std::vector<Eigen::MatrixXd>& normal_2s,
                                std::vector<Eigen::MatrixXd>& center_1s,
                                std::vector<Eigen::MatrixXd>& center_2s,
                                std::vector<double>& radius_1s,
                                std::vector<double>& radius_2s,
                                std::vector<int>& object_id_1s, 
                                std::vector<int>& object_id_2s,
                                std::vector<int>& support_types);
    // Version2s
    /*
    transform is the transform of the local transform of the feature space,
    transform distance is the distance between two feature space, 
    this distance has different meanings in different support systems.
     */
    void FeatureForOptimization(std::vector<Eigen::MatrixXd>& transform_1s,
                                std::vector<Eigen::MatrixXd>& transform_2s,
                                std::vector<double>& transform_distances,
                                std::vector<int>& object_id_1s,
                                std::vector<int>& object_id_2s,
                                std::vector<int>& support_types);

private:
	int current_object_id_;
	std::string object_register_file_;
    std::string current_working_path_;
	// mask data
	std::map<int, bool> object_deprecated_;
	std::map<std::string, std::vector<bool> > feature_deprecated_;  // Plane& Surf
	// precise data
	std::map<std::string, std::vector<bool> > feature_approximated_;
	
	// plane data & feature for optimization
	Eigen::MatrixXd plane_normals_;
	Eigen::MatrixXd plane_central_points_;
	Eigen::MatrixXd plane_boundary_points_;
    Eigen::MatrixXd opt_plane_normals_;
	Eigen::MatrixXd opt_plane_central_points_;

	// surf data & feature for optimization
	Eigen::MatrixXd surf_directions_;
	Eigen::MatrixXd surf_cenetral_points_;
	Eigen::MatrixXd surf_boundary_points_;
	Eigen::MatrixXd surf_boundary_directions_;
	Eigen::MatrixXd surf_radius_;
    Eigen::MatrixXd opt_surf_directions_;
	Eigen::MatrixXd opt_surf_cenetral_points_;

	// diff datas
	// angular data
	Eigen::MatrixXd angle_diff_plane2plane_;
	Eigen::MatrixXd angle_diff_plane2surf_;

	// dist data
	Eigen::MatrixXd dist_diff_plane2plane_;
	Eigen::MatrixXd dist_diff_plane2surf_;
	Eigen::MatrixXd dist_diff_surf2surf_;

	// objects data
	std::map<int, Eigen::MatrixXd> object_gravity_centers_;
	std::map<int, std::vector<int> > plane_feature_index_object_;
	std::map<int, std::vector<int> > surf_feature_index_object_;
    std::map<int, Eigen::MatrixXd> object_poses_;  // log poses
    std::map<int, std::string> object_names_;
    std::map<int, Eigen::MatrixXd> object_inner_transform_;

	// feature index to object id log
	std::vector<int> plane_feature2id;
	std::vector<int> surf_feature2id;

	Eigen::MatrixXd gravity_pose_;  // Used to transform z-axis to gravity
	Eigen::Vector3d gravity_direction_;  // the gravity representation

	// geometry_relation
	// If feature i support feature j, fs[i][j] will have a positive value
	// all features are saved here [plane, surf]
	Eigen::MatrixXd feature_supporting_;
	Eigen::MatrixXd plane_feature_supporting_;  
	Eigen::MatrixXd surf_feature_supporting_;
    std::map<int, bool> object_supported_;
	
	// counting data
	int num_of_object_;   
	int num_of_plane_;  
	int num_of_surf_;

	configuru::Config object_register_config_;

	// Final geometry relationship
	std::map<int, std::vector<std::pair<int, int> > > object_relationship_;
};

#endif // !SCENE_INFERENCE