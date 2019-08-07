function [num_of_plane, plane_feature_in_object] = ...
    plane_visualization(plane_feature, object_pose, color, line_width)
% PLANE_VISIALIZATION
% here the object_pose is the relative pose to the world
rotation_pose = object_pose(1:3, 1:3);
transition_pose = object_pose(1:3, 4);

num_of_plane = size(plane_feature.plane_approximated, 1);
plane_feature_in_object = (rotation_pose * plane_feature.plane_boundary_points' ...
    + repmat(transition_pose, 1, 4 * num_of_plane))';

for i = 1:1:num_of_plane
    plane_boundary_points = plane_feature_in_object(4 * i -3:4 * i, :);	
    plane_draw(plane_boundary_points, line_width, color)
end

end

