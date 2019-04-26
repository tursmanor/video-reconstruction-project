% Dataset generation that has an associated valid area of movement that
% changes at each time increment for each non-shooting camera
close all; clearvars;

% TODO
% intersecting polygons

%% Setup vars
n = 10;
k = 4;
sceneSize = [0 10 0 10];
sensorWidth = 2;
fAll = [1 1 1 1]; 
frameCount = [30 120];

% dataset structure
dataset = struct('frame',0,'pos',zeros(2,3),'f',zeros(1,2),'gtCam',0);

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
areas = {graph() graph() graph() graph()};
prevAreaRadius = zeros(1,k);

%Init position of camera at shot 1 point in an arc facing the set
[pos,f] = initCamPosition(dataset(1).gtCam,fAll,sensorWidth);
dataset(1).pos = pos;
dataset(1).f = f;
areas{dataset(1).gtCam} = addnode(areas{dataset(1).gtCam},num2str(pos(:,2)'));

for curShot = 2:n
    curCam = dataset(curShot).gtCam;
    prevCam = dataset(curShot-1).gtCam;
    
    % reset area of previous camera
    areas{prevCam} = graph();
    areas{prevCam} = addnode(areas{prevCam},num2str(dataset(curShot-1).pos(:,2)'));
    prevAreaRadius(prevCam) = 0;
    
    % For each (camera != previous camera) that is in the scene, expand its
    % corresponding area based on the previous framecount
    for i=cameras(cameras~=prevCam)
        
        ind = getPrevCamShot(i,curShot,dataset);
        
        if (ind ~= 0)
            prevPos = dataset(ind).pos;
            radius = dataset(prevCam).frame / 30;
            prevAreaRadius(i) = prevAreaRadius(i) + radius;
            areas{i} = RRT(areas{i},100,0.5,sceneSize,[],prevAreaRadius(i),prevPos(:,2)');
        end   
    end
    
    % If curCam is new, init its position in the arc, otherwise pick a
    % random point in the area of curCam
    ind = getPrevCamShot(curCam,curShot,dataset);
    if (ind == 0)
        [pos,f] = initCamPosition(curCam,fAll,sensorWidth);
        areas{curCam} = addnode(areas{curCam},num2str(pos(:,2)'));
    else
        [pos,f] = makeCamPosition(curCam,fAll,sensorWidth,areas,sceneSize);
    end
  
    % While (newPosition is in prevCam’s FOV) OR (prevCam is in newCam’s
    % FOV), get new random point
    
    dataset(curShot).pos = pos;
    dataset(curShot).f = f;
    if (findnode(areas{curCam},num2str(pos(:,2)')) == 0)
        areas{curCam} = addnode(areas{curCam},num2str(pos(:,2)'));
    end

    
    % visualize results for current frame for debugging
    color = ['r' 'b' 'g' 'c'];
    figure(1);
    axis(sceneSize); hold on;
    scatter(pos(1,2),pos(2,2),'k'); hold on;
    for i=1:4
        if (size(areas{i}.Nodes) ~= 0)
            pts = getBoundsFromGraph(areas{i});
            fill(pts(:,1),pts(:,2),color(i),'FaceAlpha',0.3); hold on;
        end
    end
   curCam
   prevAreaRadius
   pause;
end

%% Refactor numbering
% % refactor camera numbering so that it aligns with clustering expectations
% assignments = zeros(2,k);
% assignments(1,1) = dataset(1).gtCam;
% assignments(2,1) = 1;
% assignments(1,2) = dataset(2).gtCam;
% assignments(2,2) = 2;
% newCam = 3;
% for curShot = 3:n
%     if dataset(curShot).gtCam ~= assignments(1,:)
%         assignments(1,newCam) = dataset(curShot).gtCam;
%         assignments(2,newCam) = newCam;
%         newCam = newCam + 1;
%     end  
% end
% 
% % assign to dataset
% for curShot = 1:n
%    curGT = dataset(curShot).gtCam;
%    indx = find(assignments(1,:) == curGT);
%    newGT = assignments(2,indx); 
%    dataset(curShot).gtCam = newGT;
% end
% 
% save('datasetv2','dataset');

%% Helpers

% get boundary polygon from graph
function [boundPts] = getBoundsFromGraph(G)
% take all nodes of degree 1 to make polygon
nodeDeg = degree(G);

% check if there's only the one node
if(nodeDeg == 0)
    boundPts = str2num(cell2mat(G.Nodes{1,1}));

else    
    degInd = find(nodeDeg == 1);
    nodes = G.Nodes{degInd,1};
    pts = [];
    
    for i=1:size(nodes,1)
        curPt = str2num(cell2mat(nodes(i)));
        pts = [pts; curPt];
    end
    
    boundInd = boundary(pts(:,1),pts(:,2));
    boundPts = pts(boundInd,:);
    boundPts = boundPts(1:end-1,:);
end

end

% generate a random camera position
function [pos,f] = makeCamPosition(cam,fAll,sensorWidth,areas,sceneSize)

% get random point in polygon
boundPts = getBoundsFromGraph(areas{cam});
pt = randPointInPolygon(boundPts,sceneSize);

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

pos = [(pt(1)-sensorWidth/2) pt(1) (pt(1)+sensorWidth/2); pt(2) pt(2) pt(2)];
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
function [pos,f] = initCamPosition(cam,f,sensorWidth)

if (cam == 1)
    position = [1 6];
elseif (cam == 2)
    position = [4 4];
elseif (cam == 3)
    position = [6 4];
else
    position = [9 6];
end
pos = [(position(1) - sensorWidth/2) position(1) (position(1) + sensorWidth/2);
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

% newArea = incrementArea(position, curArea, constraint, velocity, time)
% 
%     If curArea exists
%         If curArea intersects with constraint
%             Remove intersection from curArea
% 
%         For each point along the circumference of curArea
%             For m evenly spaced rays (every pi/4) around the current point
%                 Increment point by velocity * time
%                 Stop early if you hit the constraint wall & only keep outermost new positions
%                 Add a new position to newArea
%     Else
%         For m evenly spaced rays (every pi/4) around position
%             Increment position by velocity*time
% Stop early if you hit the constraint wall
%             Add new position to newArea