% Dataset generation that has an associated valid area of movement that
% changes at each time increment for each non-shooting camera
%close all; clearvars;

function [dataset] = makeDatasetV2()

global nope;
nope = 0;

%% Setup vars
n = 10;
k = 4;
sceneSize = [0 10 0 10];
sensorWidth = 2;
fAll = [1 1 1 1];
frameCount = [30 120];

% dataset structure
dataset = struct('frame',0,'pos',zeros(2,3),'f',zeros(1,2),'gtCam',0,'A',cell(1,k));%,'intA',cell(10,4));

%% Init camera assignment
% generate random gt camera assignment and framelength
prevCam = 0;
for curShot = 1:n
    cam = randi([1 k]);
    
    % gt cam can't be the same as the cam in the previous shot
    while (cam == prevCam)
        cam = randi([1 k]);
    end
    
    dataset(curShot).gtCam = cam;
    prevCam = cam;
    
    frameNum = randi(frameCount);
    dataset(curShot).frame = frameNum;
end

%% Generate positions
cameras = 1:k;
areas = {[] [] [] []};

%Init position of camera at shot 1 point in an arc facing the set
[pos,f] = initCamPosition(dataset(1).gtCam,fAll,sensorWidth);
dataset(1).pos = pos;
dataset(1).f = f;
areas{dataset(1).gtCam}(1,:) = pos(:,2)';
dataset(1).A = areas;

for curShot = 2:n
    curCam = dataset(curShot).gtCam;
    prevCam = dataset(curShot-1).gtCam;
    
    % check if we made a bad replacement & fix it if so
    while (curCam == prevCam)
        curCam = randi([1 k]);
    end
    dataset(curShot).gtCam = curCam;
    
    % reset area of previous camera
    areas{prevCam} = [];
    areas{prevCam} = dataset(curShot-1).pos(:,2)';
    
    % get current constraint
    curConstraint = makeConstraint(dataset(curShot-1).pos, ...
        dataset(curShot-1).f, sceneSize);
    
    % For each (camera != previous camera) that is in the scene, expand its
    % corresponding area based on the previous framecount
    for i=cameras(cameras~=prevCam)
        ind = getPrevCamShot(i,curShot,dataset);
        if (ind ~= 0)
            radius = dataset(prevCam).frame / 30;
            areas{i} = expandPolygon(areas{i},radius,curConstraint,sceneSize);
        end
    end
    
    % If curCam is new, init its position in the arc, otherwise pick a
    % random point in the area of curCam
    [pos,f,areas] = getPositionWrapper(curCam,curShot,dataset,fAll,sensorWidth,areas,sceneSize);
    
    % While (newPosition is in prevCam’s FOV) OR (another cam is in newCam’s
    % FOV), get new random point
    % first case should never happen, so add a nope clause
    if (inpolygon(pos(1,:),pos(2,:),curConstraint(:,1),curConstraint(:,2)))
        nope = 1;
        disp('nope, FOV problem');
    end
    
    [activeCamPos] = activeCameras(k,dataset,curShot);
    
    % check if any existing camera is in the newCam's FOV
    goodPos = isGoodPosition(k,pos,f,sceneSize,activeCamPos,curCam);
    while (goodPos == 0)
        disp('Bad position blocked.');
        
        % get a new camera id
        newCam = randi([1 k]);
        while (newCam == prevCam)
            newCam = randi([1 k]);
        end
        
        dataset(curShot).gtCam = newCam;
        [pos,f,areas] = getPositionWrapper(newCam,curShot,dataset,fAll,sensorWidth,areas,sceneSize);
        goodPos = isGoodPosition(k,pos,f,sceneSize,activeCamPos,newCam);
    end
    
    % check again in case camera changed
    if (inpolygon(pos(1,:),pos(2,:),curConstraint(:,1),curConstraint(:,2)))
        nope = 1;
        disp('nope, FOV problem');
    end
    
    dataset(curShot).pos = pos;
    dataset(curShot).f = f;
    dataset(curShot).A = areas;
    
    % visualize results for current frame for debugging
    %     if(curShot > 2)
    %         color = ['r' 'b' 'g' 'c'];
    %         figure(1);
    %         clf;
    %         axis(sceneSize); hold on;
    %         fill(curConstraint(:,1),curConstraint(:,2),color(prevCam),'FaceAlpha',0.6); hold on;
    %         for i=1:4
    %             if (~isempty(areas{i}))
    %                 pts = areas{i};
    %                 fill(pts(:,1),pts(:,2),color(i),'FaceAlpha',0.3); hold on;
    %             end
    %         end
    %         pause;
    %     end
end

if (nope == 1)
    dataset = 0;
    return;
    %save('datasetv2-2','dataset');
end

%% Refactor numbering
% refactor camera numbering so that it aligns with clustering expectations
numCams = length(unique(vertcat(dataset.gtCam)));
assignments = zeros(2,numCams);
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

for curShot = 1:n
    curA = dataset(curShot).A;
    newA = cell(1,numCams);
    for i=1:numCams
        newA(assignments(2,i)) = curA(assignments(1,i));
    end
    dataset(curShot).A = newA;
end

end

%% Helpers
% returns true if current camera's FOV doesn't see any other active cameras
function [out] = isGoodPosition(k,pos,f,sceneSize,activeCamPos,curCam)

newCamFOV =  makeConstraint(pos,f,sceneSize);
results = 0;
for i=1:k
    if (i == curCam)
        continue;
    else
        tmpPos = activeCamPos(i,:);
        if(all(tmpPos == 0))
            continue;
        else
            in = inpolygon(tmpPos(1),tmpPos(2),newCamFOV(:,1),newCamFOV(:,2));
            in2 = inpolygon(tmpPos(3),tmpPos(4),newCamFOV(:,1),newCamFOV(:,2));
            in3 = inpolygon(tmpPos(5),tmpPos(6),newCamFOV(:,1),newCamFOV(:,2));
            in4 = inpolygon(tmpPos(7),tmpPos(8),newCamFOV(:,1),newCamFOV(:,2));
            results = results + in + in2 + in3 + in4;
        end
    end
end

if (results > 0)
    out = false;
else
    out = true;
end

end

% return a matrix of the current known positions for each camera
function [activeCamPos] = activeCameras(k,dataset,curShot)

activeCamPos = zeros(k,8);
for i=1:k
    ind = getPrevCamShot(i,curShot,dataset);
    if (ind == 0)
        activeCamPos(i,:) = zeros(1,8);
    else
        activeCamPos(i,1:2) = dataset(ind).pos(:,1)';
        activeCamPos(i,3:4) = dataset(ind).pos(:,2)';
        activeCamPos(i,5:6) = dataset(ind).pos(:,3)';
        activeCamPos(i,7:8) = dataset(ind).f;
    end
end

end

% wrapper for position generation
function [pos,f,areas] = getPositionWrapper(curCam,curShot,dataset,fAll,sensorWidth,areas,sceneSize)
ind = getPrevCamShot(curCam,curShot,dataset);

% get a position
if (ind == 0)
    [pos,f] = initCamPosition(curCam,fAll,sensorWidth);
    areas{curCam} = pos(:,2)';
else
    %loop until all points for the camera lie within the polygon
    curArea = areas{curCam};
    goodPoints = 0;
    while(goodPoints ~= 1)
        
        [pos,f] = makeCamPosition(curCam,fAll,sensorWidth,areas,sceneSize);
        in = inpolygon(pos(1,2),pos(2,2),curArea(:,1),curArea(:,2));
        
        if (in)
            goodPoints = 1;
        end
    end
end
end

% draw triangle to represent the FOV of the constraint camera
function [pts] = makeConstraint(pos,f,sceneSize)

pts = zeros(4,2);
pts(1,:) = f;

% get intersection with edge of scene
constraints = [pos(:,3)'; f; pos(:,1)'];
[~,slopeL,~,lLine,rLine] = makeLines(constraints);

if (slopeL > 0)
    boundX1 = [sceneSize(2)+10 sceneSize(2)+10];
    boundY1 = [sceneSize(3) sceneSize(4)+10000];
    boundX2 = [sceneSize(1)-10 sceneSize(1)-10];
    boundY2 = [sceneSize(3) sceneSize(4)+10000];
else
    boundX1 = [sceneSize(1)-10 sceneSize(1)-10];
    boundY1 = [sceneSize(3) sceneSize(4)+10000];
    boundX2 = [sceneSize(2)+10 sceneSize(2)+10];
    boundY2 = [sceneSize(3) sceneSize(4)+10000];
end

[x1,y1] = polyxpoly([pos(1,3) boundX1(1)],[pos(2,3) lLine(boundX1(1))],boundX1,boundY1);
[x2,y2] = polyxpoly([pos(1,1) boundX2(1)],[pos(2,1) rLine(boundX2(1))],boundX2,boundY2);

% debugging
if (isempty(x1) || isempty(x2))
    disp('error in makeConstraint');
    return;
end

pts(2,:) = [x1 y1];
pts(3,:) = [x2 y2];
pts(4,:) = f;

end

% generate a random camera position
function [pos,f] = makeCamPosition(cam,fAll,sWidth,areas,sceneSize)

% debugging
if(size(areas{cam},1) == 1)
    disp('error in makeCamPosition');
    return;
end

% get random point in polygon
pt = randPointInPolygon(areas{cam},sceneSize);

% figure(1);
% axis(sceneSize);
% fill(boundPts(:,1),boundPts(:,2),'r');
% hold on;
% scatter(pt(1),pt(2));
% pause(.5);
% hold on;

% bounds checking
pt(1) = min(sceneSize(2),pt(1));
pt(1) = max(sceneSize(1),pt(1));
pt(2) = min(sceneSize(4),pt(2));
pt(2) = max(sceneSize(3),pt(2));

pos = [(pt(1)-sWidth/2) pt(1) (pt(1)+sWidth/2); pt(2) pt(2) pt(2)];
f = [pt(1) pt(2) + fAll(cam)];

% randomly rotate
[camPosRot, fRot] = moveCamera(pos,f,[0;0]);
pos = camPosRot;
f = fRot';

% scatter([pos(1,:) f(:,1)],[pos(2,:) f(:,2)]);
% pause(1);
% clf;
end

% get random point in polygon
function [pt] = randPointInPolygon(polygonPts,sceneSize)
% Draw a box to contain the polygon as closely as possible, and pick a
% random point in that box
xRange = [min(polygonPts(:,1)) max(polygonPts(:,1))];
yRange = [min(polygonPts(:,2)) max(polygonPts(:,2))];

% bounds check
xRange = [max(xRange(1),sceneSize(1)) min(xRange(2),sceneSize(2))];
yRange = [max(yRange(1),sceneSize(3)) min(yRange(2),sceneSize(4))];

pt = [(xRange(1) + (xRange(1) + xRange(2)) * rand(1,1)) ...
    (yRange(1) + (yRange(1) + yRange(2)) * rand(1,1))];

% While the point is not in the polygon, generate new points
in = inpolygon(pt(1),pt(2),polygonPts(:,1),polygonPts(:,2));
while (~in)
    
    pt = [(xRange(1) + (xRange(1) + xRange(2)) * rand(1,1)) ...
        (yRange(1) + (yRange(1) + yRange(2)) * rand(1,1))];
    in = inpolygon(pt(1),pt(2),polygonPts(:,1),polygonPts(:,2));
    
end

end

% initialize cameras on first appearance in an arc facing the set
function [pos,f] = initCamPosition(cam,f,sWidth)

if (cam == 1)
    position = [1 4];
elseif (cam == 2)
    position = [4 2];
elseif (cam == 3)
    position = [6 2];
else
    position = [9 4];
end

pos = [(position(1) - sWidth/2) position(1) (position(1) + sWidth/2);
    position(2) position(2) position(2)];
f = [position(1) position(2) + f(cam)];

end

% get index of previous shot with same camera id
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
