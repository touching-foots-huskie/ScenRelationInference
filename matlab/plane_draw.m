function plane_draw(boundary_points, linewidth, color)
%draw 3d plane in forms of line
%boundary points [N, D]
x_points = boundary_points(:, 1);
y_points = boundary_points(:, 2);
z_points = boundary_points(:, 3);

%Add closed points
x_points = [x_points; x_points(1)];
y_points = [y_points; y_points(1)];
z_points = [z_points; z_points(1)];

%Plot there figures in 3d
hold on;
plot3(x_points, y_points, z_points, color, 'LineWidth', linewidth);
hold off;

end

