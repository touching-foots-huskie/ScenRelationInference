function feature_draw(feature_id_1, feature_id_2, num_of_planes, ...
    plane_boundary_points, surf_boundary_points, surf_directions, surf_radius, ...
    line_width, color)
% draw feature pairs according to feature id
% surf_threshold
surf_augment = 0.01;

% feature : 1
if feature_id_1 <= num_of_planes
    % if > num_of_planes , then it is surf & else plane
    plane_boundary_points_feature = plane_boundary_points(4 * feature_id_1 -3:4 * feature_id_1, :);
    plane_draw(plane_boundary_points_feature, line_width, color);
else
    feature_id_1 = feature_id_1 - num_of_planes;
    surf_direction_feature = surf_directions(feature_id_1, :);
    surf_boundary_points_feature = surf_boundary_points(2 * feature_id_1 -1:2 * feature_id_1, :);	
    
    surf_draw(surf_direction_feature, surf_radius(feature_id_1) + surf_augment, ...
        surf_boundary_points_feature, line_width, color);
end

% feature : 2
if feature_id_2 < num_of_planes
    % if > num_of_planes , then it is surf & else plane
    plane_boundary_points_feature = plane_boundary_points(4 * feature_id_2 -3:4 * feature_id_2, :);
    plane_draw(plane_boundary_points_feature, line_width, color);
else
    feature_id_2 = feature_id_2 - num_of_planes;
    surf_direction_feature = surf_directions(feature_id_2, :);
    surf_boundary_points_feature = surf_boundary_points(2 * feature_id_2 -1:2 * feature_id_2, :);	
    
    surf_draw(surf_direction_feature, surf_radius(feature_id_2) + surf_augment, ...
        surf_boundary_points_feature, line_width, color);
end

end

