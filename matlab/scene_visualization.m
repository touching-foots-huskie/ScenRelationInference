clc
clear 
close all
%% show scene objects & save feature together
scene_value = json_parse("test_log_4.json");

object_names = fieldnames(scene_value);
num_of_objects = size(object_names, 1);

color_list = ['b', 'g', 'y', 'k', 'm', 'c', 'b', 'g', 'y', 'k', 'm', 'c' ...
    ,'b', 'g', 'y', 'k', 'm', 'c'];

% initialization for object feature
% plane boundary_points, surf_direction, surf_boundary_points, radius
num_of_planes = 0;
plane_boundary_points = zeros(0, 3);
surf_directions = zeros(0, 3);
surf_boundary_points = zeros(0, 3);
surf_radius = [];

num_of_surfs = 0;

% get gravity pose first
object_name = 'Table_0';
object_val = getfield(scene_value, object_name);
gravity_pose = object_val.object_pose;
inverse_gravity_pose = inv(gravity_pose);

for i = 1:1:num_of_objects
    
    object_name = char(object_names(i));
    object_val = getfield(scene_value, object_name);
    
    % visualization
    object_color = color_list(i);
    [num_of_planes_in_object, num_of_surfs_in_object, plane_boundary_points_in_object, ...
        surf_boundary_points_in_object, surf_directions_in_object, surf_radius_in_object] = ...
        object_visualization(inverse_gravity_pose, object_val, object_color, 1.0);

    
    % concat feature
    num_of_planes = num_of_planes + num_of_planes_in_object;
    num_of_surfs = num_of_surfs + num_of_surfs_in_object;
    plane_boundary_points = [plane_boundary_points; plane_boundary_points_in_object];
    surf_directions = [surf_directions; surf_directions_in_object];
    surf_boundary_points = [surf_boundary_points; surf_boundary_points_in_object];
    surf_radius = [surf_radius; surf_radius_in_object];
    
end

%% show features
for i = 1:1:num_of_objects
    object_name = char(object_names(i));
    object_val = getfield(scene_value, object_name);
    object_relation = object_val.object_feature_relation;
    num_of_relationship = size(object_relation, 1);
    
    for j = 1:1:num_of_relationship
        feature_id_1 = object_relation(j, 1) + 1;
        feature_id_2 = object_relation(j, 2) + 1;
        
        % draw it
        figure;
        title(strcat(object_name, ' | ', int2str(feature_id_1), ' : ',  int2str(feature_id_2)), 'Interpreter', 'none');
    
        % draw the scene first
        for k = 1:1:num_of_objects
    
            object_name_ = char(object_names(k));
            object_val_ = getfield(scene_value, object_name_);

            % visualization
            object_color_ = color_list(k);
            object_visualization(inverse_gravity_pose, object_val_, object_color_, 1.0);

        end
        
        % then add feature
        feature_draw(feature_id_1, feature_id_2, num_of_planes, ...
            plane_boundary_points, surf_boundary_points, surf_directions, surf_radius, ...
            2.5, 'r');
    end
end

axis equal
