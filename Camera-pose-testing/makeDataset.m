close all; clearvars;

%% Dataset creation
% n shots, assume all have 30 frames
% each frame has a camera position (two 2d points) and a focal length (one
% point)
% k cameras
% sceneSize is set size

n = 10;
frameCount = 30;
k = 4;
sceneSize = [2 9; 2 9]; % middle point can't be at 1 or 10
sensorWidth = 2;

dataset = struct('frame',zeros(frameCount,1),'pos',zeros(frameCount * 2,3), ...
    'f',zeros(frameCount,2),'gtCam',0);
prevCam = 0;

fAll = [1 1 1 1]; % focal lengths of all four cameras

for curShot = 1:n
    
    cam = randi([1 k]);
    
    % gt cam can't be the same as the cam in the previous shot
    while (cam == prevCam)
        cam = randi([1 k]);
    end
    
    % assumes camera is still with some jiggling for each shot
    position = [randi(sceneSize(1,:)) randi(sceneSize(2,:))];
    camPos = [(position(1) - sensorWidth/2) position(1) (position(1) + sensorWidth/2);
        position(2) position(2) position(2)];
    f = [position(1) position(2) + fAll(cam)];
    
    % randomly rotate before adding noise
    [camPosRot, fRot] = moveCamera(camPos,f,[0;0]);
    camMatrix = repmat(camPosRot,frameCount,1);
    fMatrix = repmat(fRot',frameCount,1);
    
    % generate noise
    range = [-.5 .5];
    rangeF = [-.2 .2];
    noise = range(1) + (range(2) - range(1)) * rand(size(camMatrix,1),1);
    noise = repmat(noise,1,3);
    noiseF = rangeF(1) + (rangeF(2) - rangeF(1)) * rand(size(fMatrix,1),1);
    noiseF = repmat(noiseF,1,2);
    
    dataset(curShot).frame = (1:frameCount)';
    dataset(curShot).pos = camMatrix + noise;
    dataset(curShot).f = fMatrix + noiseF;
    dataset(curShot).gtCam = cam;
    
    prevCam = cam;
    
end

save('dataset','dataset');