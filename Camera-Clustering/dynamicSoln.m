%% Dynamic setup for clustering
close all; clearvars;
%clearAllMemoizedCaches

%% Load data
global positions shotLengths fs;

load 'datasetv1.mat';
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
seqCost([1,2],0,4,10,1);
toc

[m,ind] = min(costs);
bestSeq = seqs(ind,:);

% get GT cost
gtInd = find(n == (sum(seqs == vertcat(dataset.gtCam)',2)));
gtCost = costs(gtInd);

save('dynamicV1Traj','costs','seqs','gtInd');

%% Visualize results
algoDataset = dataset;
for i=1:n
    algoDataset(i).gtCam = bestSeq(i);
end
visualizeTwoDatasets(dataset, algoDataset, 'comparisonV1DynamicTraj')

filename = strcat('Results/','dynamicV1Traj','.gif');
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
global positions fs;

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
    % distance traveled
    if (time == inf)
        cost = -inf;
    else
        d = 0;
        for i=1:(size(pos,1)-1)
            d = d + norm(pos(i+1,:) - pos(i,:));
        end
        cost = d;
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
    
    startPos = positions(indx,:);
    startPos = startPos(3:4);
    goalPos = positions(curSize,:);
    goalPos = goalPos(3:4);
    unitVec = (goalPos - startPos) / norm(goalPos - startPos);
    
    curTime = shotLengths(curSize-1) / 30;
        
    % get current constraint as a polygon
    constr = makeConstraint(positions(curSize-1,:),fs(curSize-1,:),[0 10 0 10]);
        
    % get endpoint of straight line path for current shot
    tmpEnd = startPos + (curTime * unitVec);
    
    [x,y] = polyxpoly([startPos(1) tmpEnd(1)],[startPos(2) tmpEnd(2)],constr(:,1),constr(:,2));
    intersection = [x y];
    
    if (isempty(intersection))
        % cap at end goal
        if (norm(tmpEnd - startPos) >= norm(goalPos - startPos))
            cost = norm(goalPos - startPos);
            %disp('Made it to goal');
        else
            cost = -inf;
        end
    else
        %disp('Intersected constraint');
        cost = -inf;
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
        
        additionalCost = checkFOV([curSeq i]);
        seqCost([curSeq i],curCost + nextCost + additionalCost,k,n,memo);
        
    end
end

end
