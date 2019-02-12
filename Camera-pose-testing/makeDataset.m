function dataset = makeDataset(frameCount,pNoise)
%% Dataset creation
% n shots, between 30 and 150 frames per second (1-5 seconds)
% each frame has a camera position (two 2d points) and a focal length (one
% point)
% k cameras
% sceneSize is set size
% camera can only move within circle rad 4 between frames
% cameras must all be pointing at the set, aka the top line of the scene
% hard code initial positions of all 4 cameras to spread them out
n = 10;
vMax = 1;
aMax = 1;
k = 4;
sceneSize = [0 10; 0 10];
sensorWidth = 2;
fAll = [1 1 1 1]; % focal lengths of all four cameras
camPositions = zeros(k*2,3); % keep track of where cameras are in space
makeLine =@(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

% dataset structure
dataset = struct('frame',0,'pos',zeros(frameCount(1) * 2,3), ...
    'f',zeros(frameCount(1),2),'gtCam',0);

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
    
    % determine number of frames in this shot
    frameNum = randi(frameCount);
    dataset(curShot).frame = frameNum;
    badPosition = 1;
    if curShot ~= 1, prevCam = dataset(curShot-1).gtCam; end
    
    % set radius based on previous shot length
    if (curShot > 1)
       radius = calcRadius(dataset(curShot-1).frame / 30);
    else
        radius = 0;
    end
    
    while (badPosition)
        
        curCam = dataset(curShot).gtCam;
        
        while (curCam == prevCam)
            curCam = randi([1 k]);
            dataset(curShot).gtCam = curCam;
        end
        
        indx = getPrevCamShot(curCam,curShot,dataset);
        [pos,f] = makeCameraPosition(indx,radius,sceneSize,dataset,fAll,sensorWidth,curCam,frameNum);
        [pos,f] = addNoise(pos,f);
        [avgP,avgF] = avgCamera(pos,f);
        
        % check that we're not in the fov of the active camera-- if we are,
        % change gt camera and run again
        if (curShot > 1)

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
    
    % testing
    tmp = getPrevCamShot(dataset(curShot).gtCam,curShot,dataset);
    if (tmp ~= 0)
        [tmpPos,~] = avgCamera(dataset(tmp).pos,dataset(tmp).f);
        if (norm(tmpPos(1:2,2) - pos(1:2,2))> calcRadius(dataset(curShot-1).frame/30))
            disp('BAD');
        end
    end
    
    
    dataset(curShot).pos = pos;
    dataset(curShot).f = f;
end

% refactor camera numbering so that it aligns with clustering expectations
assignments = zeros(2,k);
assignments(1,1) = dataset(1).gtCam;
assignments(2,1) = 1;
assignments(1,2) = dataset(2).gtCam;
assignments(2,2) = 2;
newCam = 3;
for curShot = 3:n
    if dataset(curShot).gtCam ~= assignments(1,:)
        assignments(1,newCam) = dataset(curShot).gtCam;
        assignments(2,newCam) = newCam;
        newCam = newCam + 1;
    end  
end

% assign to dataset
for curShot = 1:n
   curGT = dataset(curShot).gtCam;
   indx = find(assignments(1,:) == curGT);
   newGT = assignments(2,indx); 
   dataset(curShot).gtCam = newGT;
end

%save('dataset-radius-test','dataset');

function [radius] = calcRadius(time)
    
x =[-0.0162    0.1837   -0.0038   -0.07];
radius = x(1)*time^3 + x(2)*time^2 + x(3)*time + x(4);
if radius < 0, radius = 0; end

end

function [out] = isBadPosition(curPos, constrP, constrF)
% check if the current position is within the constraint
% if so, returns true, otherwise returns false
% assumes curPos is a 2x4 matrix where the first three entries are the
% position and the fourth is the focal length

constraints = [constrP(:,3)'; constrF; constrP(:,1)'];
[pt,slopeL,slopeR,~,~] = makeLines(constraints);

% cap slope to avoid inf with vertical lines
if (abs(slopeL) > 10), slopeL = -100; end
if (abs(slopeR) > 10), slopeR = 100; end

c = [curPos(2,:) - (slopeL * (curPos(1,:) - pt(1))) - pt(2);   % left line
     curPos(2,:) - (slopeR * (curPos(1,:) - pt(1))) - pt(2)];   % right line

if (slopeL > 0)
    c(1,:) = c(1,:) * -1; 
end
if (slopeR < 0)
    c(2,:) = c(2,:) * -1; 
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
        position = [4 4];
    elseif (cam == 3)
        position = [6 4];
    else
        position = [9 6];
    end
else
    [prevPos,~] = avgCamera(dataset(prevInd).pos,dataset(prevInd).f);
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


if (prevInd ~= 0)
if (norm([prevPos(1,2) prevPos(2,2)] - [position(1) position(2)])> calcRadius(dataset(curShot-1).frame/30))
    disp('BAD');
end
end

% randomly rotate before adding noise, if the camera has appeared before
% only
if (prevInd ~= 0)
    [camPosRot, fRot] = moveCamera(camPos,f,[0;0]);
    camMatrix = repmat(camPosRot,frameCount,1);
    fMatrix = repmat(fRot',frameCount,1);
else
    camMatrix = repmat(camPos,frameCount,1);
    fMatrix = repmat(f,frameCount,1);
end

pos = camMatrix;
f = fMatrix;

end

function [pos,f] = addNoise(pos,f,pNoise)
        
    % 0.01 works
    range = [-pNoise pNoise];
    rangeF = [-.2 .2];
    noise = range(1) + (range(2) - range(1)) * rand(size(pos));
    noiseF = rangeF(1) + (rangeF(2) - rangeF(1)) * rand(size(f));
    
    % add noise
    pos = pos + noise;
    f = f + noiseF;
    
end

end