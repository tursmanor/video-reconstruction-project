close all; clearvars;

%% Dataset creation
% n shots, assume all have 30 frames
% each frame has a camera position (two 2d points) and a focal length (one
% point)
% k cameras
% sceneSize is set size
% camera can only move within circle rad 4 between frames
% cameras must all be pointing at the set, aka the top line of the scene
% hard code initial positions of all 4 cameras to spread them out
n = 10;
frameCount = 30;
k = 4;
sceneSize = [2 9; 2 9]; % middle point can't be at 1 or 10
sensorWidth = 2;
fAll = [1 1 1 1]; % focal lengths of all four cameras
camPositions = zeros(k*2,3); % keep track of where cameras are in space
radius = 2;
makeLine =@(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

% dataset structure
dataset = struct('frame',zeros(frameCount,1),'pos',zeros(frameCount * 2,3), ...
    'f',zeros(frameCount,2),'gtCam',0);

% generate random gt camera assignment first
prevCam = 0;
for curShot = 1:n
    cam = randi([1 k]);
    
    % gt cam can't be the same as the cam in the previous shot
    while (cam == prevCam)
        cam = randi([1 k]);
    end
    
    dataset(curShot).gtCam = cam;
    prevCam = cam;
end

% calculate possible positions based on radius of allowed movement
for curShot = 1:n
    
    dataset(curShot).frame = (1:frameCount)';
    badPosition = 1;
    if curShot ~= 1, prevCam = dataset(curShot-1).gtCam; end
    
    while (badPosition)
        
        curCam = dataset(curShot).gtCam;
        indx = getPrevCamShot(curCam,curShot,dataset);
        [pos,f] = makeCameraPosition(indx,radius,sceneSize,dataset,fAll,sensorWidth,curCam,frameCount);
        [avgP,avgF] = avgCamera(pos,f);
        
        % check that we're not in the fov of the active camera-- if we are,
        % change gt camera and run again
        if (curShot > 1)
            
            curCam
            prevCam
            
            [constrP,constrF] = avgCamera(dataset(curShot-1).pos,dataset(curShot-1).f);
            badPosition = isBadPosition([avgP avgF'],constrP,constrF);
            
            % if current pos is good, make sure no other cameras are in its
            % fov
            if (~badPosition)
                blocked = 0;
                
                for cam = 1:k
                    if (cam == curCam)
                        continue;
                    end
                    
                    indx = getPrevCamShot(cam,curShot,dataset);
                    
                    % current cam is on the set
                    if (indx ~= 0)
                        [camPos,camF] = avgCamera(dataset(indx).pos,dataset(indx).f);
                        blocked = blocked + isBadPosition([camPos camF'],avgP,avgF);
                        
                        % testing
                        %                          fig = figure(1);
                        %                          [~] = drawCamera(camPos,makeLine,[0 10 0 10],camF,'red'); hold on;
                        %                          [~] = drawCamera(avgP,makeLine,[0 10 0 10],avgF,'blue'); hold on;
                        %                          axis([0 10 0 10]);
                        %                          pause(2);
                        %                          clf(fig);
                        
                    else
                        continue;
                    end
                end
                
                if (blocked > 0)
                    disp('blocked');
                    badPosition = 1;
                end
                
            end
              
            % change gt camera if current pos is bad
            if (badPosition)
                newGtCam = randi([1 k]);
                
                while (newGtCam == prevCam)
                    newGtCam = randi([1 k]);
                end
                
                dataset(curShot).gtCam = newGtCam;
            end
            
        else
            break;
        end
        
    end
    
    dataset(curShot).pos = pos;
    dataset(curShot).f = f;
    
    %testing
    %   if (indx ~= 0)
    %       figure;
    %       scatter(dataset(indx).pos(1,2),dataset(indx).pos(2,2)); hold on;
    %       viscircles([dataset(indx).pos(1,2),dataset(indx).pos(2,2)],radius); hold on;
    %       scatter(dataset(curShot).pos(1,2),dataset(curShot).pos(2,2));
    %       pause(1);
    %   end
    
end

save('dataset-test-2','dataset');

function [out] = isBadPosition(curPos, constrP, constrF)
% check if the current position is within the constraint
% if so, returns true, otherwise returns false
% assumes curPos is a 2x4 matrix where the first three entries are the
% position and the fourth is the focal length

constraints = [constrP(:,3)'; constrF; constrP(:,1)'];
[pt,slopeL,slopeR,~,~] = makeLines(constraints);

if (slopeL > 0), slopeL = slopeL * -1; end
if (slopeR < 0), slopeR = slopeR * -1; end

c = [curPos(2,:) + (slopeL * (curPos(1,:) - pt(1))) - pt(2);   % left line
    curPos(2,:) - (slopeR * (curPos(1,:) - pt(1))) - pt(2)];   % right line

% check if near vertical lines-- only have to deal with
% vertical instead of vertical and horizontal since the fov
% lines will be perpendicular
if (abs(slopeL) > 5)
    % both x and y components need to be greater than f
    locations = sum(curPos > constrF');
    
    % at least one point is in a bad location
    if sum(locations == 2) > 0
        c(1,:) = [1 1 1 1];
    else
        c(1,:) = [-1 -1 -1 -1];
    end
elseif (abs(slopeR) > 5)
    % x must be smaller than fx and y must be larger than fy
    locationsX = curPos(1,:) < constrF(1);
    locationsY = curPos(2,:) > constrF(2);
    
    % at least one point is in a bad location
    if sum((locationsX + locationsY) == 2) > 0
        c(2,:) = [1 1 1 1];
    else
        c(2,:) = [-1 -1 -1 -1];
    end
end

% pos is bad if there isn't at least one negative value in each (x,y) point
if (sum((c(1,:) <= 0) | (c(2,:) <= 0)) >= 4)
    out = 0;
else
    out = 1;
end

end

function [indx] = getPrevCamShot(camera,curInd,dataset)
% returning 0 means there isn't a previous instance of the camera
% otherwise, returns the index of the previous instance

indx = 0;
for i=(curInd-1):-1:1
    if (dataset(i).gtCam == camera)
        indx = i;
        break;
    end
end

end

function [xOut,yOut] = randPtInCircle(x,y,rad)
% from https://www.mathworks.com/matlabcentral/answers/294-generate-random-points-inside-a-circle
a = 2*pi*rand;
r = sqrt(rand);
xOut = (rad*r)*cos(a)+x;
yOut = (rad*r)*sin(a)+y;
end

function [pos,f] = makeCameraPosition(prevInd,radius,sceneSize,dataset,fAll,sensorWidth,cam,frameCount)
% generate a new random position within radius r of the previous position
% if prevInd is 0, generate a position with no constraints
% assumes camera is still with some jiggling for each shot

if (prevInd == 0)
    % camera has not shown up in scene before
    if (cam == 1)
        position = [1 6];
    elseif (cam == 2)
        position = [3 4];
    elseif (cam == 3)
        position = [9 6];
    else
        position = [7 4];
    end
else
    prevPos = dataset(prevInd).pos;
    [x,y] = randPtInCircle(prevPos(1,2),prevPos(2,2),radius);
    
    % bounds checking
    x = min(sceneSize(1,2),x);
    x = max(sceneSize(1,1),x);
    y = min(sceneSize(2,2),y);
    y = max(sceneSize(2,1),y);
    
    position = [x,y];
end

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

% add noise
pos = camMatrix + noise;
f = fMatrix + noiseF;

end
