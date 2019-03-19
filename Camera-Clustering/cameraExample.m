close all; clearvars;
%% eg for init cameras and drawing them
% define scene size
sceneSize = [0 10 0 10];

% define line camera: [endpoint, mid, endpoint]
line = [3 4 5;
        2 2 2];
f = [4; 3];

% rotation and translation matrix
% theta in (-pi/2, pi/2)
theta = 5*pi/4;
translation = [5; 2];
R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];

% move camera to origin and rotate
shift = repmat(line(:,2),[1 3]);
centeredLine = line - shift;
rotLine = [R * centeredLine(:,1) ...
    R * centeredLine(:,2) ...
    R * centeredLine(:,3)];
centeredF = f - line(:,2);
rotF = R * centeredF;

% move back and apply translation
rotLine = rotLine + shift + repmat(translation,[1 3]);
rotF = rotF + line(:,2) + translation;

% test fov
makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

% plot results
[~] = drawCamera(line,makeLine,sceneSize,f,'red');
[~] = drawCamera(rotLine,makeLine,sceneSize,rotF,'blue');