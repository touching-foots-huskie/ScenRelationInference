function [num_of_plane, num_of_surf, plane_boundary_points, ...
    surf_boundary_points, surf_direction, surf_radius] = ...
    object_visualization(inverse_gravity_pose, object_feature, color, line_width)

% show all feature in object

% get relative pose
global_pose = inverse_gravity_pose * object_feature.object_pose * object_feature.inner_transform;

% show plane feature
if isfield(object_feature, 'plane_feature')
    plane_feature = object_feature.plane_feature;
    [num_of_plane,plane_boundary_points] = plane_visualization(plane_feature, global_pose, color, line_width);
else
    num_of_plane = 0;
    plane_boundary_points = zeros(0, 3);
end

if isfield(object_feature, 'surf_feature')
    surf_feature = object_feature.surf_feature;
    [num_of_surf, surf_boundary_points, surf_direction, surf_radius] = ...
        surf_visualization(surf_feature, global_pose, color, line_width);
else
    num_of_surf = 0;
    surf_boundary_points = zeros(0, 3);
    surf_direction = zeros(0, 3);
    surf_radius = zeros(0, 1);
end

end

