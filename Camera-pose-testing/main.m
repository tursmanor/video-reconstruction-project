close all; clearvars;

%% Setting up cameras 
% define scene size
sceneSize = [0 10 0 10];

% define line camera: [endpoint, mid, endpoint]
line = [3 4 5;
        2 2 2];
f = [4; 3];

% rotation and translation matrix
% theta in (-pi/2, pi/2)
theta = 5*pi/4;
translation = [3; 2];
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
lRange = line(1,1):sceneSize(2);
rRange = sceneSize(1):line(1,3);

% plot results
%drawCamera(line,makeLine,sceneSize,f);
%drawCamera(rotLine,makeLine,sceneSize,rotF);
%axis(sceneSize);

%% Energy minimization



