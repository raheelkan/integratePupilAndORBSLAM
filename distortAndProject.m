
function out = distortAndProject(gaze_point_3d, cameraParams, R, t)
K = cameraParams.IntrinsicMatrix';
p_r = cameraParams.RadialDistortion;
p_t = cameraParams.TangentialDistortion;
k_r = [p_r(1:2) p_t(1:2) p_r(3)];
%Radial Distortion Coeffs
k1 = k_r(1); k2 = k_r(2); k3 = k_r(5); 
%Tangential Distortion Coeffs
p1 = k_r(3); p2 = k_r(4);
%Coords in Camera Refernce Frame
xy_eu = R * gaze_point_3d + t;
%Pin-hole projection of the world coordinates
xy_dist = [xy_eu(1)/xy_eu(3); xy_eu(2)/xy_eu(3)];
r_sq = sum(xy_dist.^2);
L_r = (1+k1*r_sq+k2*r_sq^2+k3*r_sq^3);
%Distorted Coordinates
x_dist = xy_dist(1)*L_r+2*p1*xy_dist(1)*xy_dist(2)+p2*(r_sq+2*xy_dist(1)^2);
y_dist = xy_dist(2)*L_r+p1*(r_sq+2*xy_dist(2)^2)+2*p2*xy_dist(1)*xy_dist(2);
pos_xy_undis = [x_dist y_dist];
out = [K(1,1)*pos_xy_undis(1)+K(1,3) K(2,2)*pos_xy_undis(2)+K(2,3)];

end