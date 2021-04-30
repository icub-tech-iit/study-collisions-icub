load collisions_matrix_left % load data

matrix_data = double(collisions_matrix.data);

% These "slices" correspond to the actually slices that will be shown in
% the 3D graph. Make sure you are using the correct joint limit values,
% particularly for the lowest value (leftmost), along with the correct
% angle step (middle). Printing only the roll slices should be enough to
% show where the collisions are happening.

% these values print all the slices taken from the study
%xslice = [12.5:0.1:13.00];              % shoulder roll
%yslice = [-55:3:82];         % shoulder yaw
%zslice = [-93:3:17];         % shoulder pitch

% in practice, the slices corresponding to the roll are enough
xslice = [12.5:0.1:13.00];
yslice = [];
zslice = [];

[X, Y, Z] = meshgrid(12.0:0.1:13.00 ,-50:1:81 ,-88:3:13); 
% the end values might have to be tweaked, matlab often puts one extra cell
%  in the mesh. if this happens, reduce the maximum value of the range until
%  you get the correct number of entries in the mesh grid 

% this is the color map, in RGB values. From lowest value (top) to highest
% value (bottom). In this map a green color (top) would correspond to no
% collisions, with a degree of red corresponding to 1 (mid) or 2 (bottom)
% collisions.
map = [0 1 0
    0.5 0 0
    1 0 0];



fig = slice(X, Y, Z, matrix_data, xslice, yslice, zslice)    % display the slices
set(fig, 'EdgeColor', 'none');
xlabel("shoulder roll");
ylabel("shoulder yaw");
zlabel("shoulder pitch");
alpha('color');

alphamap('rampup');
alphamap('increase', .1);
colormap(map);
hold on;
%axis equal;

%NOTICE: the values below are outdated since the robot was miscalibrated!
% I leave the code commented in case we want to double check the 
% that the results of the study match the collisions verified in the
% real robot.

% defining lines corresponding to values registered by Stefano
%X_stef_right = [13.0, 8.6, 15.0, 10.4, 11.4, 15.9, 14.8, 13.3]; 
%X_stef_left = [7.3, 2.1, 7.3, 3.4, 5.4, 9.2, 8.7, 6.7];
%Y_stef_right = [0.0, 0.0, 15.6, 15.6, 29.9, 29.9, 45.5, 45.5];
%Y_stef_left = [0.0, 0.0, 15.6, 15.6, 29.9, 29.9, 45.5, 45.5];
%Z_stef_right = [-20.0, -4.6, -22.7 -7.7, -5.7, -21.4, -13.7, -8.0];
%Z_stef_left = [-25.8, -9.4, -21.6, -8.7, -10.9, -28.0, -17.4, -9.7];


%plot3(X_stef_right, Y_stef_right, Z_stef_right, '-o', 'color', 'red')

%view(0,0)