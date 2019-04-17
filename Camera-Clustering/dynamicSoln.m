%% Dynamic setup for clustering
close all; clearvars;

%% Load data
global positions shotLengths fs;

load 'Results/dataset-test2.mat';
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

%% Testing
%seq1 = [1 1 2 4];        % repeated camera
%seq2 = [1 2 3 4 3];      % straight line movement intersects constraint
%seq3 = [1 2 3 4 1];      % ok solution
%seq4 = [1 2 3 1 2];      % ok solution
%seq5 = [1 2 3 4 2];      % GT solution

% cost1 = calcCostPair([1 1],positions,shotLengths,fs);
% cost2 = calcCostPair([1 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 3],positions,shotLengths,fs);
% cost3 = calcCostPair([1 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 1],positions,shotLengths,fs);
% cost4 = calcCostPair([1 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 1],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 1 2],positions,shotLengths,fs);
% cost5 = calcCostPair([1 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2],positions,shotLengths,fs);
% cost6 = calcCostPair([1 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2 4],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2 4 1],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2 4 1 3],positions,shotLengths,fs);
% cost7 = calcCostPair([1 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2 3],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2 3 2],positions,shotLengths,fs) + ...
%         calcCostPair([1 2 3 4 2 3 2 3 2 3],positions,shotLengths,fs);
%
% if (cost1 == -inf), disp('Sequence 1 pass'), else, disp('Sequence 1 fail'), end
% if (cost2 == -inf), disp('Sequence 2 pass'), else, disp('Sequence 2 fail'), end
% if (cost3 ~= -inf), disp('Sequence 3 non inf'), else, disp('Sequence 3 fail'), end
% if (cost4 ~= -inf), disp('Sequence 4 non inf'), else, disp('Sequence 4 fail'), end
% if (cost5 ~= -inf), disp('Sequence 5 non inf'), else, disp('Sequence 5 fail'), end
% if (cost3 > cost5 && cost4 > cost5), disp('GT sequence best'), else, disp('GT sequence not best'), end
% if (cost6 < cost7), disp('Tree soln smaller'), else, disp('GT soln smaller'), end

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

%% Visualize results

% algoDataset = dataset;
% for i=1:n
%     algoDataset(i).gtCam = bestSeq(i);
% end

% visualizeTwoDatasets(dataset, algoDataset, 'comparison')

filename = strcat('Results/','test','.gif');
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
        cost = d + (alpha * abs(time - (shotLengths(endShot)/30)));
    end   
end

end

% assumes pair is at the end of the input sequence
% cost is based on traj opt based movement
function cost = calcCostPairTrajOpt(sequence)
global positions shotLengths fs;

cost = 0;

prevCam = sequence(end-1);
curCam = sequence(end);

if (prevCam == curCam)
    cost = -inf;
    return;
end

curSize = size(sequence,2);
[indx] = lastAppearance(curSize,sequence);

if (indx ~= 0)
    
    prevPos = reshape(positions(indx,:),[2,3]);
    curPos = reshape(positions(curSize,:),[2,3]);
    conPos = reshape(positions(curSize-1,:),[2,3]);
    
    prevPos = prevPos(:,2)';
    curPos = curPos(:,2)';
    conPos = [conPos(:,3)'; fs(curSize-1,:); conPos(:,1)'];
    
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
        cost = d + (alpha * abs(time - (shotLengths(curSize)/30)));
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

if (prevCam == curCam)
    cost = -inf;
    return;
end

curSize = size(sequence,2);
[indx] = lastAppearance(curSize,sequence);

if (indx ~= 0)
    
    prevPos = positions(indx,:);
    curPos = positions(curSize,:);
    conPos = positions(curSize-1,:);
    
    % set up line intersection as a system of linear equations
    aVal = @(x1,x2,y1,y2) -(x2 - x1)/(y2 - y1);
    bVal = @(x1,x2,y1,y2) (-y1 * ((x2-x1)/(y2-y1))) + x1;
    slope = @(x1,x2,y1,y2) (y2 - y1)/(x2 - x1);
    
    aValPath = aVal(prevPos(3),curPos(3),prevPos(4),curPos(4));
    bValPath = bVal(prevPos(3),curPos(3),prevPos(4),curPos(4));
    pathSlope = slope(prevPos(3),curPos(3),prevPos(4),curPos(4));
    
    aValLeft = aVal(conPos(5),fs(curSize-1,1),conPos(6),fs(curSize-1,2));
    bValLeft = bVal(conPos(5),fs(curSize-1,1),conPos(6),fs(curSize-1,2));
    aValRight = aVal(conPos(1),fs(curSize-1,1),conPos(2),fs(curSize-1,2));
    bValRight = bVal(conPos(1),fs(curSize-1,1),conPos(2),fs(curSize-1,2));
    
    ALeft =  [1 aValLeft; 1 aValPath];
    ARight = [1 aValRight; 1 aValPath];
    bLeft =  [bValLeft; bValPath];
    bRight = [bValRight; bValPath];
    
    leftIntersect = ALeft\bLeft;
    rightIntersect = ARight\bRight;
    
    % check if intersection occurs, otherwise accumulate distance
    % traveled
    if (~testIntersectionBounds(leftIntersect, pathSlope, fs(curSize-1,:), curPos(3:4), prevPos(3:4)) || ...
            ~testIntersectionBounds(rightIntersect, pathSlope, fs(curSize-1,:), curPos(3:4), prevPos(3:4)))
        cost = -inf;
        return;
    else
        % using d/v = t, where v is one m/s
        d = norm(prevPos(3:4) - positions(curSize,3:4));
        alpha = 0.1;
        cost = d + (alpha * abs(d - shotLengths(curSize)));
    end
    
end

end

% returns 0 if the intersection happens in a constraint region, 1 if not
function [valid] = testIntersectionBounds(pt, pSlope, f, pEnd,pBeg)

% infinity check-- means we have parallel lines
% bounds check-- only care if intersection occurs in our scene
if ((pt(2) == inf) || pt(2) > 10)
    valid = true;
    return;
end

% check if path is going in the pos or neg x direction
if (pBeg(1) < pEnd(1))
    right = true;
else
    right = false;
end

% intersection point is above the start of the constraint triangle
if (pt(2) > f(2))
    
    % positive slope
    if(pSlope > 0)
        
        % path is pointing right
        if(right)
            
            % point of intersection is larger in x and y than path end
            if(pt(1) > pEnd(1) && pt(2) > pEnd(2))
                valid = true;
            else
                valid = false;
            end
            
            % path is pointing left
        else
            
            % point of intersection is smaller in x and y than path end
            if(pt(1) < pEnd(1) && pt(2) < pEnd(2))
                valid = true;
            else
                valid = false;
            end
            
        end
        
        % negative slope
    elseif (pSlope < 0)
        
        % path is pointing right
        if(right)
            
            % point of intersection is larger in x and smaller in y than
            % path end
            if(pt(1) > pEnd(1) && pt(2) < pEnd(2))
                valid = true;
            else
                valid = false;
            end
            
            % path is pointing left
        else
            
            % point of intersection is smaller in x and bigger in y than
            % path end
            if(pt(1) < pEnd(1) && pt(2) > pEnd(2))
                valid = true;
            else
                valid = false;
            end
        end
        
        % straight line
    else
        
        % path is pointing right
        if(right)
            
            % point of intersection larger in x than path end
            if (pt(1) > pEnd(1))
                valid = true;
            else
                valid = false;
            end
            
            % path is pointing left
        else
            
            % point of intersection smaller in x than path end
            if (pt(1) < pEnd(1))
                valid = true;
            else
                valid = false;
            end
        end
    end
    
else
    valid = true;
end

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
        else
            nextCost = calcCostPair([curSeq i]);
        end
        
        seqCost([curSeq i],curCost + nextCost,k,n,memo);
        
    end
end

end
