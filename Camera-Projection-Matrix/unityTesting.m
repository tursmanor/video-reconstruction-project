%% Testing script for making projection work
close all; clearvars;

%% Load data
dataFolder = 'Data/Newest/';
points2Dold = loadUnityData([dataFolder 'camera0-frame0-screenVertices.txt'],'2D');
points3D = loadUnityData([dataFolder 'frame0-worldVertices.txt'],'3D');

numPts = size(points3D,1);
imagePix = [833 269];

% Modify points for matlab coordinate system
points2D = [points2Dold(:,1) abs(points2Dold(:,2)-imagePix(2))];

%% Get P
intrinsics = textbookMethod(points2D,points3D);
P = getProjection(points2D,points3D);

PGT = [2.77778 0.00000 0.00000 0.00000;
       0.00000 8.60182 0.00000 0.00000;
       0.00000 0.00000 -1.00060 -0.60018;
       0.00000 0.00000 -1.00000 0.00000];
  
EGT = [-0.17365	0.00000	0.98481	652.00000;
0.00000	1.00000	0.00000	35.00000;
0.98481	0.00000	0.17365	1093.00000;
0.00000	0.00000	0.00000	1.00000];

%% Test error for P
[dMat, reproj2D] = testP(P,points2D,points3D);
[dGT, reproj2DGT] = testP4(PGT,points2D,points3D,imagePix,EGT);

%% Plot reprojected points and GT points
figure(1);
scatter(points2D(:,1), points2D(:,2));
hold on;
scatter(reproj2DGT(:,1), reproj2DGT(:,2));
hold on;
%scatter(reproj2D(:,1),reproj2D(:,2));
%hold on;
legend('GT','Reprojected GT','Reprojected Ours');
axis equal;

%% Plot 2d points from worldToscreen on output images
img = imread([dataFolder 'camera0-frame0.png']);
figure(2);
imshow(img);
hold on;
scatter(points2D(:,1),points2D(:,2));

%% Helpers
% test reprojection error for 4x4 opengl projection matrix
function [d,reproj2D] = testP4(P, points2D, points3D, scale,E)

numPts = size(points3D,1);
points2DReprojected = zeros(numPts,3);

for j=1:numPts
    
        cur2DPt = E * P * [points3D(j,:)'; 1];
        
        % norm so third entry is one
        cur2DPt = cur2DPt ./ cur2DPt(end); 
        
        % window scale
        %M = [scale(1) 0 0 0; 0 scale(2) 0 0; 0 0 1 0; 0 0 0 1] ...
        %    * [.5 0 0 .5; 0 .5 0 .5; 0 0 1 0; 0 0 0 1];   
        %out = [0 + (1 + cur2DPt(1)) * scale(1)/2;
        %       0 + (1 - cur2DPt(2)) * scale(2)/2;
        %       0 + cur2DPt(3) * (1 - 0)];
        %   out = out ./ out(end);
        %points2DReprojected(j,:) = out';
        
 end
    
    points2DReprojected = points2DReprojected(:,1:2);
    
    % Test L2 dist
    d=0;
    for j=1:numPts
        curD = sqrt(sum((points2D(j,:) - points2DReprojected(j,:)).^2));
        d = d + curD;
    end
   
    d = d / numPts;
    
    reproj2D = points2DReprojected;
    

end