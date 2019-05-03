%% Dynamic clustring for V2 dataset
close all; clearvars;
% clearAllMemoizedCaches

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
global costs seqs memCost;
seqs = [];
costs = [];

memCost = memoize(@calcCostPairTrajOptMem);
memCost.CacheSize = 100;

tic
seqCost([1,2],0,4,10,0);
toc

[m,ind] = min(costs);
bestSeq = seqs(ind,:);

% get GT cost
gtInd = find(n == (sum(seqs == vertcat(dataset.gtCam)',2)));
gtCost = costs(gtInd);

%save('DynamicTraj','costs','seqs','gtInd');

%% Visualize results
algoDataset = dataset;
for i=1:n
    algoDataset(i).gtCam = bestSeq(i);
end
visualizeTwoDatasets(dataset, algoDataset, 'comparison')

filename = strcat('Results/','testv2','.gif');
drawResults(dataset,5,gtCost,filename);

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

% cost based on traj opt based movement
% setup of this function helps us m e m o i z e
function cost = calcCostPairTrajOptMem(initShot,endShot)

global positions shotLengths fs;

% camera has not appeared before
if initShot == 0
    cost = 0;
else
    
    prevPos = reshape(positions(initShot,:),[2,3]);
    curPos = reshape(positions(endShot,:),[2,3]);
    conPos = reshape(positions(endShot-1,:),[2,3]);
    
    prevPos = prevPos(:,2)';
    curPos = curPos(:,2)';
    conPos = [conPos(:,3)'; fs(endShot-1,:); conPos(:,1)'];
    
    % use traj opt to get path feasibility
    [optimalOut,~,pos] = pathOpt2D(prevPos,curPos,conPos);
    time = optimalOut(1);
    
    % check if constraint intersection occurs, otherwise accumulate
    % distance traveled, with penalty added based on how much over frame
    % time we go
    if (time == inf)
        cost = -inf;
        return;
    else
        d = 0;
        for i=1:(size(pos,1)-1)
            d = d + norm(pos(i+1,:) - pos(i,:));
        end
        
        alpha = 0.1;
        cost = d; %+ (alpha * abs(time - (shotLengths(endShot)/30)));
    end
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

% build a tree that keeps track of cost of each walk/sequence
function [] = seqCost(curSeq,curCost,k,n,memo)

global costs seqs memCost;

% base cases
if (curCost == -inf)
    return;
    
elseif (size(curSeq,2) == n)
    costs = [costs curCost];
    seqs = [seqs; curSeq];
    % testing
    %size(costs)
    
    % recursive case
else
    prevCam = curSeq(end);
    nextCam = 1:min(max(curSeq)+1,k);
    nextCam = nextCam(nextCam~=prevCam);
    
    for i = nextCam
        
        if (memo == 1)
            curLen = length(curSeq) + 1;
            nextCost = memCost(lastAppearance(curLen,[curSeq i]),curLen);
        else
            nextCost = calcCostPair([curSeq i]);
        end
        
        seqCost([curSeq i],curCost + nextCost,k,n,memo);
        
    end
end
end
