%% test surf_draw function
% surf_direction = [0.2, 0.3, 1.0];
% surf_direction = surf_direction / norm(surf_direction);
% radius = 1.0;
% boundary_points = [0.0, 0.0, 0.0;
%                    0.4, 0.6, 2.0];
% linewidth = 2.0;
% color = 'r';
% surf_draw(surf_direction, radius, boundary_points, linewidth, color);

%% test json decoding
% decode related scene condition
file_name = "test_log.json";
val = json_parse(file_name);

% show according geometry based on json file
