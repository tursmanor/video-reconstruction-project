%% Dynamic setup for clustering
close all; clearvars;

% To-do
% 4. visualize results

%% Load data
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

cost1 = calcCostPair([1 1],positions,shotLengths,fs);
cost2 = calcCostPair([1 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 3],positions,shotLengths,fs);
cost3 = calcCostPair([1 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 1],positions,shotLengths,fs);
cost4 = calcCostPair([1 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 1],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 1 2],positions,shotLengths,fs);
cost5 = calcCostPair([1 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2],positions,shotLengths,fs);
cost6 = calcCostPair([1 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2 4],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2 4 1],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2 4 1 3],positions,shotLengths,fs);
cost7 = calcCostPair([1 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2 3],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2 3 2],positions,shotLengths,fs) + ...
        calcCostPair([1 2 3 4 2 3 2 3 2 3],positions,shotLengths,fs);   
    
if (cost1 == -inf), disp('Sequence 1 pass'), else, disp('Sequence 1 fail'), end
if (cost2 == -inf), disp('Sequence 2 pass'), else, disp('Sequence 2 fail'), end
if (cost3 ~= -inf), disp('Sequence 3 non inf'), else, disp('Sequence 3 fail'), end
if (cost4 ~= -inf), disp('Sequence 4 non inf'), else, disp('Sequence 4 fail'), end
if (cost5 ~= -inf), disp('Sequence 5 non inf'), else, disp('Sequence 5 fail'), end
if (cost3 > cost5 && cost4 > cost5), disp('GT sequence best'), else, disp('GT sequence not best'), end
if (cost6 < cost7), disp('Tree soln smaller'), else, disp('GT soln smaller'), end

%% Build tree
global costs seqs;
seqs = [];
costs = [];

seqCost([1,2],0,positions,shotLengths,fs,4,10);

[m,ind] = min(costs);
bestSeq = seqs(ind,:);

%% Visualize results

algoDataset = dataset;
for i=1:n
    algoDataset(i).gtCam = bestSeq(i);
end

visualizeTwoDatasets(dataset, algoDataset, 'comparison')

%% Helpers 

% only looks at one pair of assignments at a point in the sequence
% assume that pair is at the end of the input sequence
function cost = calcCostPair(sequence,positions,shotLengths,fs)
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
         cost = cost + d + (alpha * abs(d - shotLengths(curSize)));
     end
     
 end
 
end

% Calculate cost of a given camera sequence
%
% Output: 
% cost         - cost of camera sequence
%
% Input:
% sequence     - sequence of camera assignments
% positions    - positions associated with each shot
% shotLengths  - lengths associated with each shot
% fs           - focal lengths associated with each shot
function [cost] = calcCost(sequence,positions,shotLengths,fs)
    cost = 0;
    
    % Costs:
    % -inf if same camera appears twice in a row
    % -inf if straight line movement passes through constraint region
    % accumulate distance traveled as cost
    % if travel time is more than shot length, penalize by alpha
    prevCam = sequence(1);
    for i=2:length(sequence)
        
        curCam = sequence(i);
        if (prevCam == curCam)
            cost = -inf;
            return;
        end
            
        % check if cur camera has appeared before, if so, see if its path
        % intersects the constraint region-- in other words, see if its
        % path intersects either of the lines defining the constraint
        [indx] = lastAppearance(i,sequence);
        
        if (indx ~= 0)
            
            prevPos = positions(indx,:);
            curPos = positions(i,:);
            conPos = positions(i-1,:);
            
            % set up line intersection as a system of linear equations
            aVal = @(x1,x2,y1,y2) -(x2 - x1)/(y2 - y1);
            bVal = @(x1,x2,y1,y2) (-y1 * ((x2-x1)/(y2-y1))) + x1;
            slope = @(x1,x2,y1,y2) (y2 - y1)/(x2 - x1);
            
            aValPath = aVal(prevPos(3),curPos(3),prevPos(4),curPos(4));
            bValPath = bVal(prevPos(3),curPos(3),prevPos(4),curPos(4));
            pathSlope = slope(prevPos(3),curPos(3),prevPos(4),curPos(4));
            
            aValLeft = aVal(conPos(5),fs(i-1,1),conPos(6),fs(i-1,2));
            bValLeft = bVal(conPos(5),fs(i-1,1),conPos(6),fs(i-1,2));
            aValRight = aVal(conPos(1),fs(i-1,1),conPos(2),fs(i-1,2));
            bValRight = bVal(conPos(1),fs(i-1,1),conPos(2),fs(i-1,2));
            
            ALeft =  [1 aValLeft; 1 aValPath];
            ARight = [1 aValRight; 1 aValPath];
            bLeft =  [bValLeft; bValPath];
            bRight = [bValRight; bValPath];
            
            leftIntersect = ALeft\bLeft;
            rightIntersect = ARight\bRight;
            
            % check if intersection occurs, otherwise accumulate distance
            % traveled
            if (~testIntersectionBounds(leftIntersect, pathSlope, fs(i-1,:), curPos(3:4), prevPos(3:4)) || ...
                    ~testIntersectionBounds(rightIntersect, pathSlope, fs(i-1,:), curPos(3:4), prevPos(3:4)))
                cost = -inf;
                return;
            else
                % using d/v = t, where v is one m/s
                d = norm(prevPos(3:4) - positions(i,3:4));
                alpha = 0.1;
                cost = cost + d + (alpha * abs(d - shotLengths(i)));
            end
            
        end
        
        prevCam = curCam;
        
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
function [] = seqCost(curSeq,curCost,positions,shotLengths,fs,k,n)

global costs seqs;

% base cases
if (curCost == -inf)
    return;
    
elseif (size(curSeq,2) == n)
   costs = [costs curCost];
   seqs = [seqs; curSeq];

% recursive case
else
    prevCam = curSeq(end);
    nextCam = 1:min(max(curSeq)+1,k);
    nextCam = nextCam(nextCam~=prevCam);
    
    for i = nextCam
        nextCost = calcCostPair([curSeq i],positions,shotLengths,fs);
        seqCost([curSeq i], curCost + nextCost, ...
                positions,shotLengths,fs,k,n);
    end
end

end
