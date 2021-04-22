load collisions_matrix_rough                     % load data

matrix_data = double(collisions_matrix.data);

% These "slices" correspond to the actually slices that will be shown in
% the 3D graph. Make sure you are using the correct joint limit values,
% particularly for the lowest value (leftmost), along with the correct
% angle step (middle). Printing only the roll slices should be enough to
% show where the collisions are happening.

% these value below correspond to a lower joint limit of 7 with a degree
% step of 3.0
xslice = [7:3:16];              % shoulder roll
yslice = []%[-55:3:82];         % shoulder yaw
zslice = []%[-93:3:17];         % shoulder pitch

[X, Y, Z] = meshgrid(7:3:162 ,-55:3:82 ,-93:3:17); 
% the end values might have to be tweaked, matlab often puts one extra cell
%  in the mesh if this happens, reduce the maximum value of the range until
%  you get the correct number of entries in the mesh grid 

% this is the color map, in RGB values. From lowest value (top) to highest
% value (bottom). 
map = [0 1 0
    0.5 0 0
    1 0 0];


slice(X, Y, Z, matrix_data, xslice, yslice, zslice)    % display the slices
xlabel("shoulder roll");
ylabel("shoulder yaw");
zlabel("shoulder pitch");

colormap(map);