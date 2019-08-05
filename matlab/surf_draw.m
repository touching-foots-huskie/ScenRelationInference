function surf_draw(surf_direction, radius, boundary_points, linewidth, color)
%draw surf in terms of line
%boundary points [N, D]
%surf_direction [1, D]
%radius 1

hold on;

%% Draw the cycle
% draw the cycle
z_axis = [0, 0, 1];

% get transform from z-axis to surf_direction.
if(norm(z_axis - surf_direction) < 1e-3)
    rotation_matrix = eye(3);
else
    theta = acos(dot(z_axis, surf_direction));
    
    k = cross(z_axis, surf_direction);
    k = k / norm(k);
    
    k_upper = [0, -k(3), k(2);
               k(3), 0, -k(1);
               -k(2), k(1), 0];
           
    rotation_matrix = eye(3) * cos(theta) + (1-cos(theta)) * k' * k +...
        sin(theta) * k_upper;
end


start_point = boundary_points(1, :)';
end_point = boundary_points(2, :)';

% original cycle
cycle_t = 0 : 0.01 : 2*pi;
cycle_x = radius * cos(cycle_t);
cycle_y = radius * sin(cycle_t);

cycle_p = [cycle_x; cycle_y; zeros(size(cycle_x))];
cycle_p = rotation_matrix * cycle_p;

% start cycle
point_num = size(cycle_p, 2);
cycle_p_start = cycle_p + repmat(start_point, 1, point_num);
cycle_p_end = cycle_p + repmat(end_point, 1, point_num);

start_x_points = cycle_p_start(1, :);
start_y_points = cycle_p_start(2, :);
start_z_points = cycle_p_start(3, :);

plot3(start_x_points, start_y_points, start_z_points, color, 'LineWidth', linewidth);

end_x_points = cycle_p_end(1, :);
end_y_points = cycle_p_end(2, :);
end_z_points = cycle_p_end(3, :);

plot3(end_x_points, end_y_points, end_z_points, color, 'LineWidth', linewidth);
%% Plot the line
% draw the related line
x_points = boundary_points(:, 1);
y_points = boundary_points(:, 2);
z_points = boundary_points(:, 3);


plot3(x_points, y_points, z_points, color, 'LineWidth', linewidth);

hold off

axis equal
end
