% Dataset generation that has an associated valid area of movement that
% changes at each time increment for each non-shooting camera
close all; clearvars;

% TODO
% --

%% Setup vars
n = 10;
k = 4;
sceneSize = [0 10 0 10];
sensorWidth = 2;
fAll = [1 1 1 1];
frameCount = [30 120];

% dataset structure
dataset = struct('frame',0,'pos',zeros(2,3),'f',zeros(1,2),'gtCam',0,'A',{[] [] [] []});

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
    % first case should never happen
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
        
        [pos,f,areas] = getPositionWrapper(newCam,curShot,dataset,fAll,sensorWidth,areas,sceneSize);
        dataset(curShot).gtCam = newCam;
        goodPos = isGoodPosition(k,pos,f,sceneSize,activeCamPos,newCam);    
    end
     
    dataset(curShot).pos = pos;
    dataset(curShot).f = f;
    dataset(curShot).A = areas;
    
    % visualize results for current frame for debugging
    if(curShot > 2)
        color = ['r' 'b' 'g' 'c'];
        figure(1);
        clf;
        axis(sceneSize); hold on;
        fill(curConstraint(:,1),curConstraint(:,2),color(prevCam),'FaceAlpha',0.6); hold on;
        for i=1:4
            if (~isempty(areas{i}))
                pts = areas{i};
                fill(pts(:,1),pts(:,2),color(i),'FaceAlpha',0.3); hold on;
            end
        end
        pause;
    end
end

%% Refactor numbering
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

save('datasetv2','dataset');

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
            results = results + in;
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

activeCamPos = zeros(k,2);
for i=1:k
    ind = getPrevCamShot(i,curShot,dataset);
    if (ind == 0)
        activeCamPos(i,:) = [0 0];
    else
        activeCamPos(i,:) = dataset(ind).pos(:,2)';
    end
end

end

% wrapper for position generation
function [pos,f,areas] = getPositionWrapper(curCam,curShot,dataset,fAll,sensorWidth,areas,sceneSize)
    ind = getPrevCamShot(curCam,curShot,dataset);
    if (ind == 0)
        [pos,f] = initCamPosition(curCam,fAll,sensorWidth);
        areas{curCam} = pos(:,2)';
    else
        [pos,f] = makeCamPosition(curCam,fAll,sensorWidth,areas,sceneSize);
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
    boundY1 = [sceneSize(3) sceneSize(4)+1000];
    boundX2 = [sceneSize(1)-10 sceneSize(1)-10];
    boundY2 = [sceneSize(3) sceneSize(4)+1000];
else
    boundX1 = [sceneSize(1)-10 sceneSize(1)-10];
    boundY1 = [sceneSize(3) sceneSize(4)+1000];
    boundX2 = [sceneSize(2)+10 sceneSize(2)+10];
    boundY2 = [sceneSize(3) sceneSize(4)+1000];
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

% sample points every theta degrees of a circle centered at [center]  with
% radius [radius] to create a polygon
function [pts] = samplePoints(center, radius)

theta = pi/4;
pts = [];
curAngle = 0;
while (curAngle < (2*pi))
    
    x = radius*cos(curAngle)+center(1);
    y = radius*sin(curAngle)+center(2);
    
    pts = [pts; x y];
    curAngle = curAngle + theta;
    
end

end

% expand each edge point of the previous polygon to make a new, bigger, and
% better polygon
function [output]= expandPolygon(curPolygon,radius,constraint,sceneSize)

allPts = [];
makeLine =@(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

% draw a new circle around each existing polygon pt
for i = 1:size(curPolygon,1)   
    startPt = curPolygon(i,:);
    pts = samplePoints(startPt,radius);
    allPts = [allPts; pts];   
end

boundInd = boundary(allPts(:,1),allPts(:,2),0.2);
newPolygon = allPts(boundInd,:);

% bounds clamping
newPolygon(newPolygon(:,1) < sceneSize(1),1) = sceneSize(1);
newPolygon(newPolygon(:,1) > sceneSize(2),1) = sceneSize(2);
newPolygon(newPolygon(:,2) < sceneSize(3),2) = sceneSize(3);
newPolygon(newPolygon(:,2) > sceneSize(4),2) = sceneSize(4);

% take intersection with constraint
[x,y] = polyxpoly(newPolygon(:,1),newPolygon(:,2),constraint(:,1),constraint(:,2));
intersection = [x y];
numInt = size(intersection,1);

if (isempty(intersection) || numInt == 1)
    output = newPolygon;
elseif (numInt > 2)
    disp('nope')
    output = newPolygon;
else
    
    % three nonempty intersection cases: +- sloped line cut (results in 1 new
    % polygon), v cut (resulting in 2 new polygons)
    % pick vertex, check if it's in the constraint. if it is, pick new vertex
    for i=1:size(newPolygon,1)
        startPt = newPolygon(i,:);
        startInd = i;
        in = inpolygon(startPt(1),startPt(2),constraint(:,1),constraint(:,2));
        if (~in), break; end
    end
    
    % cycle through vertices, and remove those that fall between the constraint
    % intersections-- this is the scenario where there are only two
    % intersection points -- need another case for 4 intersection points
    curPt = startPt;
    curInd = startInd;
    newPolygon2 = curPt;
    inConstraint = 0;
    for i=1:size(newPolygon,1)
        
        curInd = mod(curInd + 1,size(newPolygon,1));
        if (curInd == 0), curInd = size(newPolygon,1); end
        
        nextPt = newPolygon(curInd,:);
        
        % check if an intersection point lies on the line segment between
        % the current and next point. if it does, remove if from the list
        % of intersection points
        approxEqual = 0;
        if(numInt > 0)
            for j=1:numInt
                curIntPt = intersection(j,:);
                y = makeLine(curIntPt(1),curPt(1),curPt(2),nextPt(1),nextPt(2));
                approxEqual = abs(curIntPt(2) - y) <= 0.0001;
                
                % vertical line check
                if ((nextPt(1) - curPt(1)) == 0 && (curIntPt(1) == curPt(1)))
                    maxPt = max(curPt(2),nextPt(2));
                    minPt = min(curPt(2),nextPt(2));
                    if (curIntPt(2) <= maxPt && curIntPt(2) >= minPt)
                        approxEqual = 1;
                    else
                        approxEqual = 0;
                    end
                end
                
                % horizontal line check
                if ((nextPt(2) - curPt(2)) == 0 && (curIntPt(2) == curPt(2)))
                    maxPt = max(curPt(1),nextPt(1));
                    minPt = min(curPt(1),nextPt(1));
                    if (curIntPt(1) <= maxPt && curIntPt(1) >= minPt)
                        approxEqual = 1;
                    else
                        approxEqual = 0;
                    end
                end
                
                if (approxEqual)
                    inConstraint = inConstraint + 1;
                    newPolygon2 = [newPolygon2; curIntPt];
                    intersection(j,:) = [];
                    numInt = numInt - 1;
                    break;      
                end
            end
        end
        
        % if so, nextPt is not added to the new polygon, and we throw up a flag
        if (inConstraint == 0)
            newPolygon2 = [newPolygon2; nextPt];
        elseif (inConstraint == 1)
            % add the tip of the FOV if it's inside the current polygon
            in = inpolygon(constraint(1,1),constraint(1,2),newPolygon(:,1),newPolygon(:,2));
            if(in)
                newPolygon2 = [newPolygon2; constraint(1,:)];
            end
        elseif (inConstraint == 2)
            % turn off the flag after we've passed the constraint a second time
            inConstraint = 0;
            newPolygon2 = [newPolygon2; nextPt];
        end
        
        curPt = nextPt;
    end
    
    output = newPolygon2;  

    badInd = [];
    % pruning stray edges
    for i=1:size(output,1)
        curPt = output(i,:);
        [in,on] = inpolygon(curPt(1),curPt(2),constraint(:,1),constraint(:,2));
        if(in && ~on)
            badInd = [badInd i];
        end
    end
    output(badInd,:) = [];

end

% figure(3);
% clf;
% fill(output(:,1),output(:,2),'r','FaceAlpha',0.2); hold on;
% axis([0 10 0 10]); hold on;
% scatter(curPolygon(:,1),curPolygon(:,2)); hold on;
% fill(constraint(:,1),constraint(:,2),'b','FaceAlpha',0.2);
% pause;

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
    position = [1 6];
elseif (cam == 2)
    position = [4 4];
elseif (cam == 3)
    position = [6 4];
else
    position = [9 6];
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
