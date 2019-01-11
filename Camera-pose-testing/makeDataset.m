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
makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

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
    
    while (badPosition)
       
        curCam = dataset(curShot).gtCam;
        indx = getPrevCamShot(curCam,curShot,dataset);
        [pos,f] = makeCameraPosition(indx,radius,sceneSize,dataset,fAll,sensorWidth,curCam,frameCount);
        [avgP,avgF] = avgCamera(pos,f);
      
        % check that we're not in the fov of the active camera-- if we are,
        % change gt camera and run again
        if (curShot > 1)
            
            [constrP,constrF] = avgCamera(dataset(curShot-1).pos,dataset(curShot-1).f);
            constraints = [constrP(:,3)';constrF;constrP(:,1)'];
            [pt,~,~,slopeL,slopeR,~,~] = makeLines(constraints);

            c = [max(avgP(2,:),pt(2)) + (-slopeL * (avgP(1,:) - pt(1))) - pt(2);     % left line
                max(avgP(2,:),pt(2)) - (slopeR * (avgP(1,:) - pt(1))) - pt(2)];   % right line

            % check if near vertical lines-- only have to deal with
            % vertical instead of vertical and horizontal since the fov
            % lines will be perpendicular
            if (slopeL > 10)
                if all(avgP(:,1) < constrF') || (avgP(2,1) > constrF(2))
                    c(1,1) = -1;
                end
                 if all(avgP(:,2) < constrF') || (avgP(2,2) > constrF(2))
                    c(1,2) = -1;
                 end
                 if all(avgP(:,3) < constrF') || (avgP(2,3) > constrF(2))
                    c(1,3) = -1;
                end
            elseif (slopeR > 10)
                if all(avgP(:,1) < constrF') || (avgP(2,1) > constrF(2))
                    c(2,1) = -1;
                end
                 if all(avgP(:,2) < constrF') || (avgP(2,2) > constrF(2))
                    c(2,2) = -1;
                 end
                 if all(avgP(:,3) < constrF') || (avgP(2,3) > constrF(2))
                    c(2,3) = -1;
                 end
            end
            
            % testing
%             fig = figure(1);
%             [~] = drawCamera(avgP,makeLine,[0 10 0 10],avgF,'red'); hold on;
%             [~] = drawCamera(constrP,makeLine,[0 10 0 10],constrF,'blue'); hold on;
%             axis([0 10 0 10]);
%             pause(3);
%             clf(fig);
            
            % change gt camera if current pos is bad, where the pos is bad
            % if there isn't at least one negative value in each (x,y) point
            if (sum((c(1,:) <= 0) + (c(2,:) <= 0)) >= 3)
                badPosition = 0;
                %disp('GOOD');
            else
                %disp('BAD');
                newGtCam = randi([1 k]);
                
                while (newGtCam == dataset(curShot-1).gtCam)
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

save('dataset','dataset');

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
        position = [1 8];
    elseif (cam == 2)
        position = [5 5];
    elseif (cam == 3)
        position = [9 8];
    else
        position = [5 1];
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
