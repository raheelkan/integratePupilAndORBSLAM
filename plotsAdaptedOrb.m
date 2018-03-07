%-----------------------------------------------------------------------
% 1-point RANSAC EKF SLAM from a monocular sequence
%-----------------------------------------------------------------------

%% Top Left
figure(figure_all);
subplot(im_fig);
hold off;
imagesc(undistortImage(im, cameraParams));
colormap gray;
hold on;
title('Current Image with OrbFeatures');
if ~isempty(points)
    plot(points(1,:), points(2,:), 'gs', 'LineWidth',1, 'MarkerSize',8, 'MarkerEdgeColor','g'); %, 'MarkerFaceColor',[0.5,0.5,0.5]
    plot(points(1,:), points(2,:), 'gd', 'MarkerSize',2, 'MarkerFaceColor',[0,1,0]);
end


gaze_point = norm_pos_out(currentIm,:);
gaze_point = [gaze_point(1)*imSizePupil(2) (1-gaze_point(2))*imSizePupil(1)];
plot(gaze_point(1), gaze_point(2),'cx','Markersize',15, 'LineWidth', 4);

%%Plot the 3d gaze point (gaze_points_3d_out) on the current image
gaze_point_3d = gaze_points_3d_out(currentIm,:)/100;%%Convert Centimeters to meters

% rotCam = pose(1:3,1:3)'; %Rinv = pose(1:3,1:3); ==> R = Rinv';
% cameraCentre = -pose(1:3,1:3)*pose(1:3, end); % C = -R'*t = -Rinv*t
rotCam = pose(1:3,1:3)'; %Rinv = pose(1:3,1:3); ==> R = Rinv';
cameraCentre = -pose(1:3,1:3)'*pose(1:3, end); % C = -R'*t = -Rinv*t

gaze_point_3d_wc = rotCam*(gaze_point_3d') + cameraCentre; %%% ==inv(r)*(rotated_gaze - loca(1:3))
% P = K_world*pose(1:3,1:3)*[eye(3) pose(1:3, end)];
gp_proj = distortAndProject(gaze_point_3d_wc, cameraParams, pose(1:3,1:3), pose(1:3, end));%pose(1:3,1:3)*
% P = K_world*[pose(1:3,1:3) pose(1:3,1:3)*pose(1:3, end)];%
% gp_proj = P*[gaze_point_3d; 1];
% gp_proj = [gp_proj(1)/gp_proj(3) gp_proj(2)/gp_proj(3)];

% imagePoints = worldToImage(cameraParams,pose(1:3,1:3)',pose(1:3, 4),gaze_point_3d', 'ApplyDistortion',true);

plot(gp_proj(1), gp_proj(2),'rx','Markersize',15, 'LineWidth', 4);
 
 distFeat_GP = (gp_proj(1)-points(1,:)).^2+(gp_proj(2)-points(2,:)).^2;
 nearestFeatureIdx = find(distFeat_GP==min(distFeat_GP));
 nearestFeature3DCoords = map(nearestFeatureIdx, :);
 nearestFeature3DCoords = nearestFeature3DCoords(1,:); %% Same feature listed twice in Map
 plot(points(1,nearestFeatureIdx), points(2,nearestFeatureIdx),'yo','Markersize',12, 'LineWidth', 2);%, 'MarkerFaceColor',[0.5,0.5,0.5]
%  scaleFactor = sqrt(sum(gaze_point_3d'.^2))/sqrt(sum((nearestFeature3DCoords'-cameraCentre).^2));
%  scaleFactor = sqrt(sum(gaze_point_3d'.^2))/sqrt(sum((gaze_point_3d_wc-cameraCentre).^2));
%  scaleFactor3D = gaze_point_3d./nearestFeature3DCoords
 
%  plot(imagePoints(1), imagePoints(2),'gx','Markersize',15, 'LineWidth', 4);
% 
axes_handler = get(gcf,'CurrentAxes');
set(axes_handler,'XTick',[],'YTick',[]);

    description = ['PUPIL: gaze_x = ' num2str(gaze_point_3d(1)) ', gaze_y = ' num2str(gaze_point_3d(2)) ', gaze_z = ' num2str(gaze_point_3d(3))];
    text(10, 2*535, description)
    text(10, 2*550, ['Distance = ' num2str(sqrt(sum(gaze_point_3d.^2))) ' meters'])
if ~isempty(nearestFeature3DCoords)
    depth_error = sqrt(sum((gaze_point_3d-nearestFeature3DCoords).^2));
    description = ['ORBSLAM: gaze_x = ' num2str(nearestFeature3DCoords(1)) ', gaze_y = ' num2str(nearestFeature3DCoords(2)) ', gaze_z = ' num2str(nearestFeature3DCoords(3))];
    text(2*410, 2*535, description)
    text(2*410, 2*550, ['Depth Error = ' num2str(depth_error) ' m'])
    text(2*510, 2*550, ['Distance: nearestFeat-cameraCentre = ' num2str(sqrt(sum((nearestFeature3DCoords'-cameraCentre).^2))) ' m'])
end

%% Top Right
figure(figure_all);
subplot(near3D_fig);
hold off;

scatter3(ref(:,1), ref(:,2), ref(:,3),'g.');
hold on; scatter3(map(:,1), map(:,2), map(:,3),'r.'); 
plot3(map(nearestFeatureIdx,1), map(nearestFeatureIdx,2), map(nearestFeatureIdx,3),'mo','Markersize',15) 

drawCameraOrb( pose, 'k' );

quat = rotm2quat(rotCam);
indTraj = currentIm-startFrame+1;
trajectory(:, indTraj) = [cameraCentre; quat']; %Camera Centre= -R'*t
plot3( trajectory(1, 1:indTraj), trajectory(2, 1: indTraj),...
    trajectory(3, 1:indTraj), 'k', 'LineWidth', 2 );

plot3(gaze_point_3d_wc(1),gaze_point_3d_wc(2),gaze_point_3d_wc(3),'mx','Markersize',15)
text(gaze_point_3d_wc(1),gaze_point_3d_wc(2),gaze_point_3d_wc(3),'gp','HorizontalAlignment','left');

title('Top View: X-Z. Green: Ref Map, Red: Current Features');
% axes_handler = get(gcf,'CurrentAxes');
% axis([-3 3 -1 1 -1 2]);
axis([-3 3  -3 3 -1 7]);
grid on;
view(0,0); % XZ

%% Bottom Left
figure(figure_all);
subplot(near3D_fig2);
hold off;
% map = [10 -2 -2 15; 10 2 -2.5 15; 10 -2 1.5 15; 10 2 1 15]
% ref = [10 -2 -2 15; 10 2 -2.5 15; 10 -2 1.5 15; 10 2 1 15]
scatter3(ref(:,1), ref(:,2), ref(:,3),'g.');
hold on; scatter3(map(:,1), map(:,2), map(:,3),'r.'); 
plot3(map(nearestFeatureIdx,1), map(nearestFeatureIdx,2), map(nearestFeatureIdx,3),'mo','Markersize',15) 


drawCameraOrb( pose, 'k' );

% % quat = rotm2quat(pose(1:3, 1:3));
% % indTraj = currentIm-startFrame+1;
% % trajectory(:, indTraj) = [(-pose(1:3,1:3)*pose(1:3, end))' quat]'; %Camera Centre= -R'*t
plot3( trajectory(1, 1:indTraj), trajectory(2, 1: indTraj),...
    trajectory(3, 1:indTraj), 'k', 'LineWidth', 2 );

plot3(gaze_point_3d_wc(1),gaze_point_3d_wc(2),gaze_point_3d_wc(3),'mx','Markersize',15)
text(gaze_point_3d_wc(1),gaze_point_3d_wc(2),gaze_point_3d_wc(3),'gp','HorizontalAlignment','left');

title('Camera/User View: X-(-Y). Green: Ref Map, Red: Current Features');
% axes_handler = get(gcf,'CurrentAxes');
% axis([-3 3 -1 1 -1 2]);
axis([-3 3  -3 3 -1 7]);
grid on;
view(0,-90);% X(-Y) axes

%% Bottom Right

% % figure(figure_all);
% % subplot(near3D_fig3);
% % hold off; 
% % 
% % scatter3(ref(:,1), ref(:,2), ref(:,3),'g.');
% % hold on; scatter3(map(:,1), map(:,2), map(:,3),'r.'); 
% % plot3(map(nearestFeatureIdx,1), map(nearestFeatureIdx,2), map(nearestFeatureIdx,3),'mo','Markersize',15) 
% % 
% % drawCameraOrb( pose, 'k' );
% % hold on;
% % 
% % quat = rotm2quat(pose(1:3, 1:3));
% % indTraj = currentIm-startFrame+1;
% % trajectory(:, indTraj) = [(-pose(1:3,1:3)*pose(1:3, end))' quat]'; %Camera Centre= -R'*t
% % plot3( trajectory(1, 1:indTraj), trajectory(2, 1: indTraj),...
% %     trajectory(3, 1:indTraj), 'k', 'LineWidth', 2 );
% % % set(gca, 'CameraPosition', (-pose(1:3,1:3)*pose(1:3, end))')
% % 
% % plot3(gaze_point_3d_wc(1),gaze_point_3d_wc(2),gaze_point_3d_wc(3),'mx','Markersize',15)
% % text(gaze_point_3d_wc(1),gaze_point_3d_wc(2),gaze_point_3d_wc(3),'gp','HorizontalAlignment','left');
% % 
% % title('Top View: Y-Z. Camera Trajectory');
% % % axes_handler = get(gcf,'CurrentAxes');
% % % axis([-3 3 -1 1 -1 2]);
% % axis([-3 3  -3 3 -1 7]);
% % grid on;
% % % view(90,0)  % YZ axes
% % view(0,0); % XZ


figure(figure_all);
subplot(near3D_fig3);
hold off;
imagesc(undistortImage(im, cameraParams));
colormap gray;
hold on;
title('All map points in current FOV');
if ~isempty(points_map)
    plot(points_map(1,:), points_map(2,:), 'gs', 'LineWidth',1, 'MarkerSize',8, 'MarkerEdgeColor','g'); %, 'MarkerFaceColor',[0.5,0.5,0.5]
    plot(points_map(1,:), points_map(2,:), 'gd', 'MarkerSize',2, 'MarkerFaceColor',[0,1,0]);
end


% gaze_point = norm_pos_out(currentIm,:);
% gaze_point = [gaze_point(1)*imSizePupil(2) (1-gaze_point(2))*imSizePupil(1)];
plot(gaze_point(1), gaze_point(2),'cx','Markersize',15, 'LineWidth', 4);

%%Plot the 3d gaze point (gaze_points_3d_out) on the current image
% gaze_point_3d = gaze_points_3d_out(currentIm,:)/100;%%Convert Centimeters to meters
% 
% % rotCam = pose(1:3,1:3)'; %Rinv = pose(1:3,1:3); ==> R = Rinv';
% % cameraCentre = -pose(1:3,1:3)*pose(1:3, end); % C = -R'*t = -Rinv*t
% rotCam = pose(1:3,1:3)'; %Rinv = pose(1:3,1:3); ==> R = Rinv';
% cameraCentre = -pose(1:3,1:3)'*pose(1:3, end); % C = -R'*t = -Rinv*t
% 
% gaze_point_3d_wc = rotCam*(gaze_point_3d') + cameraCentre; %%% ==inv(r)*(rotated_gaze - loca(1:3))
% % P = K_world*pose(1:3,1:3)*[eye(3) pose(1:3, end)];
% gp_proj = distortAndProject(gaze_point_3d_wc, cameraParams, pose(1:3,1:3), pose(1:3, end));%pose(1:3,1:3)*
% % P = K_world*[pose(1:3,1:3) pose(1:3,1:3)*pose(1:3, end)];%
% % gp_proj = P*[gaze_point_3d; 1];
% % gp_proj = [gp_proj(1)/gp_proj(3) gp_proj(2)/gp_proj(3)];
% 
% % imagePoints = worldToImage(cameraParams,pose(1:3,1:3)',pose(1:3, 4),gaze_point_3d', 'ApplyDistortion',true);

plot(gp_proj(1), gp_proj(2),'rx','Markersize',15, 'LineWidth', 4);
 
%  distFeat_GP = (gp_proj(1)-points(1,:)).^2+(gp_proj(2)-points(2,:)).^2;
%  nearestFeatureIdx = find(distFeat_GP==min(distFeat_GP));
%  nearestFeature3DCoords = map(nearestFeatureIdx, :);
%  nearestFeature3DCoords = nearestFeature3DCoords(1,:); %% Same feature listed twice in Map
 plot(points(1,nearestFeatureIdx), points(2,nearestFeatureIdx),'yo','Markersize',12, 'LineWidth', 2);%, 'MarkerFaceColor',[0.5,0.5,0.5]
%  scaleFactor = sqrt(sum(gaze_point_3d'.^2))/sqrt(sum(nearestFeature3DCoords.^2));
%  scaleFactor3D = gaze_point_3d./nearestFeature3DCoords
 
%  plot(imagePoints(1), imagePoints(2),'gx','Markersize',15, 'LineWidth', 4);
% 
axes_handler = get(gcf,'CurrentAxes');
set(axes_handler,'XTick',[],'YTick',[]);