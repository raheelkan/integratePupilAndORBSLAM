
clc; clear; close all;
% function integrateGazePointsWithORBSLAM()
PUPIL_PATH = '/home/ga47qiv/codes/OrbSLAM_codes/recordings_640x480_DS2_ORB_PTAM/';
DATASET_PATH = '/home/ga47qiv/codes/OrbSLAM_codes/recordings_640x480_DS2_ORB_PTAM/000/sceneImages/';
ORB_SLAM_PATH = '/home/ga47qiv/codes/OrbSLAM_codes/recordings_640x480_DS2_ORB_PTAM/000/';
scaleFactorMap  = 1; %1.2427*3.0079;%0.1021;%0.0702;%1; %0.0476; 
startFrame = 660;%645;%596;%261;
addpath(genpath(PUPIL_PATH))
addpath(genpath(DATASET_PATH))
addpath(genpath(ORB_SLAM_PATH))

load('cameraParams_640x480.mat')
%% Gaze Point Calculations (From PUPIL Exports)
filename = 'gaze_positions.csv';
% K_tum = [517.306408 0 318.643040; 0 516.469215 255.313989; 0 0 1];
K_world = [399.2863, 0.0, 328.9676;  % For 640x480
    0.0, 396.6767, 247.2154;
    0.0, 0.0, 1.0];
%Radial and Tangential distortions [k1 k2 p1 p2 k3] where kn: radial distortion coeffs and pn:
%tangential distortion coeffs
distortions = [-0.4330; 0.1990; 0.0021; 0.0015; -0.0445];
usePupilCalib = true;
imSizePupil = [480 640];%cameraParams.ImageSize;
[norm_pos_out, gaze_points_3d_out, eye_centre0_3d_out, eye_centre1_3d_out, gaze_normal0_out, gaze_normal1_out] = convertGaze2camCoordsTop(filename, K_world, distortions, imSizePupil, usePupilCalib);
gaze_points_calc = findGazePoints(eye_centre0_3d_out, eye_centre1_3d_out, gaze_normal0_out, gaze_normal1_out);
eul = [];
%% OrbSLAM Stuff
sequenceFolder = sprintf('%s%s',DATASET_PATH,'rgb/*.png');
imagefiles = dir(sequenceFolder);      
nfiles = length(imagefiles);    % Number of files found

fPose = sprintf('%04d%s.csv',startFrame,'_Pose');
fMap = sprintf('%04d%s.csv',startFrame,'_Map');
fRef = sprintf('%04d%s.csv',startFrame,'_Ref');
pose = importPose(fPose);
map = importMap(fMap);
ref = importRef(fRef);
imageNamePre = '/home/ga47qiv/codes/OrbSLAM_codes/recordings_640x480_DS2_ORB_PTAM/000/sceneImages/rgb/';
set_plots;
for currentIm=startFrame:nfiles-10
%     gazePosition = norm_pos_out(currentIm,:);
   imageName = sprintf('%s%s',imageNamePre,imagefiles(currentIm).name);%imageNamePre + imagefiles(i).name;
   im = imread(imageName);
   
   fPose = sprintf('%04d%s.csv',currentIm,'_Pose');
   fMap = sprintf('%04d%s.csv',currentIm,'_Map');
   fRef = sprintf('%04d%s.csv',currentIm,'_Ref');
   scaleFactorMap  = 1; %1.2427*3.0079;%0.1021;%0.0702;%1; %0.0476; 
%    scaleFactor3D   =  [0.1502   -3.6921    0.0118];%[1 1 1]; %[0.0075    0.5428    0.2997];
   pose = importPose(fPose);
   map = importMap(fMap);
   ref = importRef(fRef);
   map = scaleFactorMap.*map(:,2:4);
   ref = scaleFactorMap.*ref(:,2:4);
   % % P = K_world*[Rinv' Rinv'*Tcw(1:3,end)];%
   pose = reshape(pose(1,:), [4 4])';
   pose(1:3,end) = scaleFactorMap'.*pose(1:3,end);
  
%% Actual Cal (OrbSLAM Code)
% Pose contains [R | Rinv*t] and we need to convert ot to P = [R | t]
% % Tcw = reshape(pose(1,:), [4 4])';
% % Rinv = reshape(pose(2,1:9), [3 3])';
% % camCentre = (pose(3,1:3))'; 

   P = K_world*[pose(1:3,1:3) pose(1:3, end)];%pose(1:3,1:3)*
   if ~isempty(map)
%         points = P*[map ones(size(map,1), 1)]';
%         points = [points(1,:)./points(3,:); points(2,:)./points(3,:)];
        points = P*[map ones(size(map,1), 1)]';
        points = [points(1,:)./points(3,:); points(2,:)./points(3,:)];
        
        points_map = P*[ref ones(size(ref,1), 1)]';
        points_map = [points_map(1,:)./points_map(3,:); points_map(2,:)./points_map(3,:)];
%         points = K_world*pose(1:3,:)*[map(:,2:4) ones(size(map,1), 1)]';
%         points = [points(1,:)./points(3,:); points(2,:)./points(3,:)];
   end
   plotsAdaptedOrb;
   
end

rmpath(genpath(PUPIL_PATH))
rmpath(genpath(DATASET_PATH))
rmpath(genpath(ORB_SLAM_PATH))

% end
