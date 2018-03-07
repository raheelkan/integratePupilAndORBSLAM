%-----------------------------------------------------------------------
% 1-point RANSAC EKF SLAM from a monocular sequence
%-----------------------------------------------------------------------

%-----------------------------------------------------------------------
function drawCameraOrb( pose, color )
% 
camera_size = 0.2;

vertices = [ 0    1 -1  0 ;
             0    0  0  0 ;
            2 -1 -1 2  ]*camera_size;
        
% Rotate the camera
r = pose(1:3,1:3)';
vertices = r * vertices;

% Translate the camera
cameraCentre = -pose(1:3,1:3)'*pose(1:3, end); % C = -R'*t
vertices = vertices + [cameraCentre cameraCentre cameraCentre cameraCentre];

% Draw the vertices
plot3(vertices(1,:), vertices(2,:), vertices(3,:), color, 'LineWidth', 2);