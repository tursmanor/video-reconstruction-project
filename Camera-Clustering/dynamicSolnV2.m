%% Dynamic clustring for V2 dataset
close all; clearvars;
clearAllMemoizedCaches

%% Load data
global positions shotLengths fs;

load 'datasetv2.mat';
n = length(dataset);
positions = zeros(n,6);
shotLengths = zeros(n,1);
fs = zeros(n,2);
numCam = 4;

for i=1:n
    [avgP,avgF] = avgCamera(dataset(i).pos,dataset(i).f);
    positions(i,:) = [avgP(:,1)' avgP(:,2)' avgP(:,3)'];
    fs(i,:) = avgF;
    shotLengths(i) = dataset(i).frame;
end

%% Build tree
global costs seqs memCost memCostRRT;
seqs = [];
costs = [];

memCostRRT = memoize(@calcCostRRT);
memCostRRT.CacheSize = 100;
memCost = memoize(@calcCostPiecewiseLine);
memCost.CacheSize = 100;

% 2 = RRT, 1 = PWLine, 0 = Line
tic
seqCost([1,2],0,4,10,2);
toc

[m,ind] = min(costs);
bestSeq = seqs(ind,:);

% get GT cost
gtInd = find(n == (sum(seqs == vertcat(dataset.gtCam)',2)));
gtCost = costs(gtInd);

save('dynamicV2RRT','costs','seqs','gtInd');

%% Visualize results
% algoDataset = dataset;
% for i=1:n
%     algoDataset(i).gtCam = bestSeq(i);
% end
% visualizeTwoDatasets(dataset, algoDataset, 'comparisonV2DynamicPWLine')
% 
% filename = strcat('Results/','dynamicV2PWLine','.gif');
% drawResults(dataset,5,gtCost,filename);

%% Helpers
% draw compact result
function [] = drawResults(dataset,plots,gtCost,filename)
global seqs costs;

n = length(dataset);
[~,ind] = mink(costs,plots);

fovHan = zeros(1,plots+1);
fig = figure(1);
set(fig,'position',[0,0,1200,700]);

for shots=1:n
    
    % draw frame 'shot' for each sequence
    for i=1:plots
        
        algoDataset = dataset;
        seq = seqs(ind(i),:);
        for j=1:n
            algoDataset(j).gtCam = seq(j);
        end
        
        subplot(2,3,i);
        fovHan(i) = compactVisualization(algoDataset,shots);
        title("Sequence = " + num2str(seq) + newline + ...
            "Cost = " + num2str(costs(ind(i))));
        hold on;
        
    end
    
    % plot GT in bottom right corner
    subplot(2,3,6);
    fovHan(end) = compactVisualization(dataset,shots);
    title("GT = " + num2str(seq) + newline + ...
        "Cost = " + num2str(gtCost));
    hold on;
    
    % save as a gif
    im = frame2im(getframe(fig));
    [A,map] = rgb2ind(im,256);
    if (shots == 1)
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime', ...
            dataset(shots).frame/30);
    else
        imwrite(A,map,filename,'gif','WriteMode','append', ...
            'DelayTime',dataset(shots).frame/30);
    end
    
    % turn off fov
    if(shots > 1)
        for i=1:plots+1
            set(fovHan(i),'Visible','off');
        end
    end
end


end

% cost based on RRT
function cost = calcCostRRT(initShot,endShot)
global positions shotLengths fs;

if (initShot == 0)
    cost = 0;
    return;
end

startPos = reshape(positions(initShot,:),[2,3]);
startPos = startPos(:,2)';
goalPos = reshape(positions(endShot,:),[2,3]);
goalPos = goalPos(:,2)';

% get number of shots camera j had to move in
% eg., for [1,2,_], 1 has 3-1-1 = 1 shot to move
numActiveShots = endShot - initShot - 1;     
    
paths = graph();
paths = addnode(paths,num2str(startPos));
maxPathLength = 0;

% increase area size based on number of active shots
for i=1:numActiveShots

    shot = initShot + i;
    curTime = shotLengths(shot) / 30;
    maxPathLength = maxPathLength + curTime;
    
    % get current constraint as a polygon
    constr = makeConstraint(positions(shot,:),fs(shot,:),[0 10 0 10]);    
    
    % populate graph via RRT
    paths = RRT(paths,100,0.01,[0 10 0 10],constr,maxPathLength,goalPos);
    
    % don't keep going through active shots if we've already hit
    % the goal
    if (findnode(paths,num2str(goalPos)) ~= 0)
        break;
    end
end

% if we reached the goal in time, get shortest path from start to goal
% of final graph
goalNodeID = findnode(paths,num2str(goalPos));
if(goalNodeID ~= 0)
    goalEdges = outedges(paths,goalNodeID);
    cost = min(paths.Edges{goalEdges,'Weight'});
else
    cost = -inf;
end
end

% cost based on piecewise line based movement
% setup of this function helps us m e m o i z e
function cost = calcCostPiecewiseLine(initShot,endShot)

global positions shotLengths fs;

if (initShot == 0)
    cost = 0;
    return;
end

startPos = reshape(positions(initShot,:),[2,3]);
startPos = startPos(:,2)';
goalPos = reshape(positions(endShot,:),[2,3]);
goalPos = goalPos(:,2)';

% get number of shots camera j had to move in
% eg., for [1,2,_], 1 has 3-1-1 = 1 shot to move
numActiveShots = endShot - initShot - 1;     
curArea = startPos;
        
paths = graph();
paths = addnode(paths,num2str(startPos));

% increase area size based on number of active shots
for i=1:numActiveShots

    shot = initShot + i;
    curTime = shotLengths(shot) / 30;
    
    % get current constraint as a polygon
    constr = makeConstraint(positions(shot,:),fs(shot,:),[0 10 0 10]);
    
    % add to path graph
    curArea = expandPolygon(curArea,curTime,constr,[0 10 0 10]);
 
    % if there is only one node in the graph, spread in a circle,
    % otherwise populate as normal
    curIn = inpolygon(goalPos(1),goalPos(2),curArea(:,1),curArea(:,2));
    
    if((size(paths.Nodes,1) == 1) && ~curIn)
        paths = initGraph(paths,curArea);
    else
        paths = populateGraph(paths,goalPos,curArea,curTime);
    end
    
    % don't keep going through active shots if we've already hit the goal
    if (curIn)
        break;
    end
end

% if we reached the goal in time, get shortest path from start to goal
% of final graph
if(findnode(paths,num2str(goalPos)) ~= 0)
    [~,cost] = shortestpath(paths,findnode(paths,num2str(startPos)), ...
        findnode(paths,num2str(goalPos)));
else
    cost = -inf;
end

end

% only looks at one pair of assignments at a point in the sequence
% assume that pair is at the end of the input sequence
% cost is based on straight line movement
function cost = calcCostPair(sequence)
global positions shotLengths fs;

cost = 0;

prevCam = sequence(end-1);
curCam = sequence(end);
curSize = size(sequence,2);
[indx] = lastAppearance(curSize,sequence);

if (prevCam == curCam)
    cost = -inf;
    return;
end

if(indx ~= 0)
    
    % get number of shots camera j had to move in
    % eg., for [1,2,_], 1 has 3-1-1 = 1 shot to move
    numActiveShots = curSize - indx - 1;
    
    startPos = positions(indx,:);
    startPos = startPos(3:4);
    goalPos = positions(curSize,:);
    goalPos = goalPos(3:4);
    unitVec = (goalPos - startPos) / norm(goalPos - startPos);
    
    % accumulate possible straight line path per active shot
    dist = 0;
    curEnd = startPos;
    for i=1:numActiveShots
        
        shot = indx + i;
        curTime = shotLengths(shot) / 30;
        
        % get current constraint as a polygon
        constr = makeConstraint(positions(shot,:),fs(shot,:),[0 10 0 10]);
        
        % get endpoint of straight line path for current shot
        tmpEnd = curEnd + (curTime * unitVec);
        
        % cap at end goal
        if (norm(tmpEnd - startPos) > norm(goalPos - startPos))
            curEnd = goalPos;
            dist = norm(goalPos - startPos);
            disp('Made it to goal');
            break;
        end
        
        % what happens first: intersection, or reaching the goal
        % check if either endpoint is in the constraint
        in = inpolygon(curEnd(1),curEnd(2),constr(:,1),constr(:,2));
        in2 = inpolygon(tmpEnd(1),tmpEnd(2),constr(:,1),constr(:,2));
        
        if (in && in2)
            dist = -inf;
            disp('Both points in constraint');
            break;
        end
        
        [x,y] = polyxpoly([curEnd(1) tmpEnd(1)],[curEnd(2) tmpEnd(2)],constr(:,1),constr(:,2));
        intersection = [x y];
        
        if (isempty(intersection))
            dist = dist + norm(tmpEnd - curEnd);
            curEnd = tmpEnd;
        else
            disp('Intersected constraint');
            curEnd = intersection;
            dist = dist + norm(intersection - curEnd);
        end
    end
    
    if (isequal(curEnd,goalPos))
        cost = dist;
    else
        cost = -inf;
    end
end

% check that nothing is in the new cam's FOV
constr = makeConstraint(positions(curSize,:),fs(curSize,:),[0 10 0 10]);
in = 0;
numCam = max(sequence);
indices = zeros(1,numCam);
for j=1:numCam
    tmpInd = find(sequence == j);
    indices(j) = tmpInd(end);
end

for j=1:numCam
    if (j == curCam)
        continue;
    end
    curPt = positions(indices(j),:);
    curPt = curPt(3:4);
    in = in + inpolygon(curPt(1),curPt(2),constr(:,1),constr(:,2));
end

if (in ~= 0)
    disp('Sequence invalid, camera in new FOV');
    cost = -inf;
end
end

% init graph spread
function [paths] = initGraph(paths,curArea)
    % add a ray from the center node to each vertex in the current area,
    % like spokes
    centerNode = paths.Nodes{1,'Name'};
    centerNode = str2num(cell2mat(centerNode));  
    for index = 1:size(curArea,1)       
        curPt = curArea(index,:);
        totalDist = norm(curPt - centerNode);
        
        % add node if it isn't already in the graph
        if (findnode(paths,num2str(curPt)) == 0)
            
            paths = addnode(paths,num2str(curPt));
            paths = addedge(paths,findnode(paths,num2str(centerNode)), ...
                findnode(paths,num2str(curPt)),totalDist);
        end
    end
end

% add to graph if we are past the initial population
function [pathsOut] = populateGraph(paths,goalPos,curArea,curTime)
% iterate over the nodes of the graph, and all vertices at the edge of the
% current area polygon. add edges to either the goal if it is within
% the current area
in = inpolygon(goalPos(1),goalPos(2),curArea(:,1),curArea(:,2));
eps = 0.0001;
pathsOut = paths;

for i = 1:size(paths.Nodes,1)
    
    curNode = paths.Nodes{i,'Name'};
    curNode = str2num(cell2mat(curNode));
    
    for j = 1:size(curArea,1)    
        % check if goal is within current area
        if(in)
            newPt = goalPos;
        else
            newPt = curArea(j,:);
        end
        
        % check if it's possible to travel to the new point in time
        dist = norm(newPt - curNode);
        if(dist <= (curTime + eps))
            % add goal node if it isn't already in the graph
            if (findnode(pathsOut,num2str(newPt)) == 0)
                pathsOut = addnode(pathsOut,num2str(newPt));
            end
            
            pathsOut = addedge(pathsOut,i,findnode(pathsOut,num2str(newPt)),dist);
        end
    end
end
end

% draw triangle to represent the FOV of the constraint camera
function [pts] = makeConstraint(pos,f,sceneSize)

pts = zeros(4,2);
pts(1,:) = f;

% get intersection with edge of scene
constraints = [pos(5:6); f; pos(1:2)];
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

[x1,y1] = polyxpoly([pos(5) boundX1(1)],[pos(6) lLine(boundX1(1))],boundX1,boundY1);
[x2,y2] = polyxpoly([pos(1) boundX2(1)],[pos(2) rLine(boundX2(1))],boundX2,boundY2);

% debugging
if (isempty(x1) || isempty(x2))
    disp('error in makeConstraint');
    return;
end

pts(2,:) = [x1 y1];
pts(3,:) = [x2 y2];
pts(4,:) = f;

end

% returns most recent instance of camera before curInd, 0 if it doesn't
% appear before curInd
function [indx] = lastAppearance(curInd,sequence)

indx = 0;
cameraId = sequence(curInd);

for i=(curInd-1):-1:1
    if (sequence(i) == cameraId)
        indx = i;
        break;
    end
end

end

% check that nothing is in the new cam's FOV
function cost = checkFOV(sequence)
global positions fs;

shot = size(sequence,2);
constr = makeConstraint(positions(shot,:),fs(shot,:),[0 10 0 10]);
in = 0;

% iterate through all active cameras, skipping the active FOV
for j=1:max(sequence)
    if (j == sequence(end))
        continue;
    end
    currentIndex = find(sequence == j,1,'last');
    curPt = positions(currentIndex,3:4);
    in = in + inpolygon(curPt(1),curPt(2),constr(:,1),constr(:,2));
end

if (in ~= 0)
    disp('Sequence invalid, camera in new FOV');
    cost = -inf;
else
    cost = 0;
end
end

% build a tree that keeps track of cost of each walk/sequence
function [] = seqCost(curSeq,curCost,k,n,memo)

global costs seqs memCost memCostRRT;

% base cases
if (curCost == -inf)
    return;
    
elseif (size(curSeq,2) == n)
    costs = [costs curCost];
    seqs = [seqs; curSeq];
    % testing
    size(costs)
    
    % recursive case
else
    prevCam = curSeq(end);
    nextCam = 1:min(max(curSeq)+1,k);
    nextCam = nextCam(nextCam~=prevCam);
    
    for i = nextCam
        
        if (memo == 1)
            curLen = length(curSeq) + 1;
            nextCost = memCost(lastAppearance(curLen,[curSeq i]),curLen);
        elseif (memo == 2)
            curLen = length(curSeq) + 1;
            nextCost = memCostRRT(lastAppearance(curLen,[curSeq i]),curLen);
        else
            nextCost = calcCostPair([curSeq i]);
        end
        
        additionalCost = checkFOV([curSeq i]);
        seqCost([curSeq i],curCost + nextCost + additionalCost,k,n,memo);
        
    end
end
end