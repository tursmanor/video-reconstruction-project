%% Testing
% Method 1: textbook implementation
% Method 3: kukelova

%% TO DO: 
% figure out what resolution the code is projecting onto in unity
% how do i make it what i want

close all; clearvars;

%% Load data
dataFolder = 'Data/Hi-Res/';
points2Dold = loadUnityData([dataFolder 'camera0-frame0-screenVertices.txt'],'2D');
points3D = loadUnityData([dataFolder 'frame0-worldVertices.txt'],'3D');
numPts = size(points3D,1);
imagePix = [1366 608];
%imagePix = [833 281];
% Modify points for matlab coordinate system?
points2D = [points2Dold(:,1) abs(points2Dold(:,2)-imagePix(2))];

%% GT camera data for camera0, frame0
% Camera information from unity
sensorMM = [25 25];
fMM = 25;
camCenterGT = [-500; 50; 200];
R = eulToRot([0 40 0]);
%sensorMM = [36 24];
%fMM = 50;
%camCenterGT = [652.0; 35.0; 1093.0];   % camera 0
%camCenterGT = [347.0; 35.0; 1695.0];    % camera 1

% Calculate focal length in pixels
fPix = (fMM .* imagePix) ./ sensorMM;
%fPix = (fMM .* imagePix) ./ sqrt(sum(sensorMM.^2));

% Calculate principal point in pixels
sensorCenterMM = sensorMM ./ 2;
ppPix = (sensorCenterMM .* imagePix) ./ sensorMM;

% Create intrinsic matrix
K = [fPix(1) 0 ppPix(1);
     0 fPix(2) ppPix(2);
     0 0 1];

%R = eulToRot([0.0 260.0 0.0]);  % camera 0 
%R = eulToRot([0 200.0 0]);      % camera 1

% P = K[R|-RC], where C is the camera center
PGT = K * [R -R*camCenterGT];
    
%% Methods
% Method 1
intrinsics = textbookMethod(points2D,points3D);
P = getProjection(points2D,points3D);

% Method 3
% Get 5 random points from input
%randPts = randperm(numPts);
%randPts = randPts(1:5);
%[intrinsics2, PMat] = kukelovaMethod(points2D(randPts,:), points3D(randPts,:));

%% Test error for P
%[dMat,~] = testP(cat(3,P,PMat),points2D,points3D);
[dMat, reproj2D] = testP(P,points2D,points3D);
%[dGT, reproj2DGT] = testP(PGT,points2D,points3D);

%% Plot reprojected points and GT points
figure(1);
scatter(points2D(:,1), points2D(:,2));
hold on;
scatter(reproj2DGT(:,1), reproj2DGT(:,2));
hold on;
%scatter(reproj2D(:,1),reproj2D(:,2));
hold on;
legend('GT','Reprojected GT','Reprojected Ours');
axis equal;

%% Plot 2d points from worldToscreen on output images
% img = imread([dataFolder 'camera1-frame498.png']);
% figure(2);
% imshow(img);
% hold on;
% scatter(points2D(:,1),points2D(:,2));

%% Test error for cam center
% dist = sqrt(sum((intrinsics.camCenterW - camCenterGT).^2));
% for i=1:size(intrinsics2,2)
%     dist = sqrt(sum((intrinsics2(i).camCenterW - camCenterGT).^2));
% end
