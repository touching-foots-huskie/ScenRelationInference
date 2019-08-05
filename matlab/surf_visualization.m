function [num_of_surf, surf_boundary_points_in_object, surf_direction_in_object, surf_radius] = ...
    surf_visualization(surf_feature, object_pose, color, line_width)

% surf_threshold
surf_augment = 0.01;
% update surf direction
rotation_pose = object_pose(1:3, 1:3);
transition_pose = object_pose(1:3, 4);

num_of_surf = size(surf_feature.surf_approximated, 1);
surf_direction_in_object = (rotation_pose * surf_feature.surf_directions')';
surf_boundary_points_in_object = (rotation_pose * surf_feature.surf_boundary_points' ... 
    + repmat(transition_pose, 1, 2 * num_of_surf))';

surf_radius = surf_feature.surf_radius;

for i = 1:1:num_of_surf
    surf_direction = surf_direction_in_object(i, :);
    surf_boundary_points = surf_boundary_points_in_object(2 * i -1:2 * i, :);	
    
    surf_draw(surf_direction, surf_radius(i) + surf_augment, ...
        surf_boundary_points, line_width, color);
end
end

