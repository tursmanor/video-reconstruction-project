%% Dynamic setup for clustering
close all; clearvars;

% To-do
% 2. calculate cost function
% 1. write test cases
% 3. test table generation
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

seq1 = [1 1 2 4];        % repeated camera
seq2 = [1 2 3 4 3];      % straight line movement intersects constraint
seq3 = [1 2 3 4 1];      % ok solution
seq4 = [1 2 3 1 2];      % ok solution
seq5 = [1 2 3 4 2];      % GT solution

cost1 = calcCost(seq1,positions,shotLengths,fs);
cost2 = calcCost(seq2,positions,shotLengths,fs);
cost3 = calcCost(seq3,positions,shotLengths,fs);
cost4 = calcCost(seq4,positions,shotLengths,fs);
cost5 = calcCost(seq5,positions,shotLengths,fs);

if (cost1 == -inf), disp('Sequence 1 pass'), else, disp('Sequence 1 fail'), end
if (cost2 == -inf), disp('Sequence 2 pass'), else, disp('Sequence 2 fail'), end
if (cost3 ~= -inf), disp('Sequence 3 non inf'), else, disp('Sequence 3 fail'), end
if (cost4 ~= -inf), disp('Sequence 4 non inf'), else, disp('Sequence 4 fail'), end
if (cost5 ~= -inf), disp('Sequence 5 non inf'), else, disp('Sequence 5 fail'), end
if (cost3 > cost5 && cost4 > cost5), disp('GT sequence best'), else, disp('GT sequence not best'), end

%% Build table
% global indexTable errorTable widthsTable;
% indexTable = [];
% errorTable = [];
% widthsTable = [];
% 
% [minError,cameraConfig] = dynamicCamAssignment(numCam,n,shotSubset,positions,shotLengths,fs);

%% Visualize results




%% Helpers 

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
    
% Calculate table
%
% Output: 
% cameraConfig - optimal camera assignnments for n shots
% minError     - min computed cost for the best assignment
%
% Input: 
% numCameras   - number of cameras in pool
% n            - number of shots
% positions    - nx2, positions associated with each shot
% shotLengths  - nx1, lengths of each shot
% shotSubset   - subset of shots
% function [minError,cameraConfig] = dynamicCamAssignment(numCameras,n,shotSubset,positions,shotLengths,fs)
% 
% global indexTable errorTable widthsTable;
% 
% % Collect an error for each camera num
% widthErrors = zeros(numCameras, 1);
% widthPatterns = cell(numCameras, 1);
% 
% for i=1:numCameras
%     
%     camWidth = 1;
%     curCam = i;
%     
%     % First, get remaining subset width after placing
%     % this lens at right-most part of epiSlice
%     upToWidth = shotSubset - camWidth;
%     
%     % If this width is negative, i.e., if the lens width is larger,
%     % then this lens cannot fit and we assign a large error
%     if upToWidth < 0
%         widthErrors(i) = Inf;
%         widthPatterns(i) = {-1};
%         % If the lens width equals the remaining subset width
%     elseif upToWidth == 0
%         % This is a base case.
%         % Compute the error directly.
%         %
%         widthErrors(i) = calcCost(upToWidth, positions(1:size(upToWidth),:),shotLengths(1:size(upToWidth)));
%         widthPatterns(i) = {curCam};
%         
%         % It is possible for a length of 'lensWidth' to already
%         % exist from a deeper level plus a lens (e.g., 10 will
%         % already exist from 5 + 5.
%         index = indexTable == camWidth;
%         if sum( index ) == 0
%             % Doesn't exist in the table; add
%             indexTable = [indexTable camWidth];
%             errorTable = [errorTable widthErrors(i)];
%             widthsTable = [widthsTable widthPatterns(i)];
%         else
%             % Replace existing value if error is less.
%             errors = errorTable;
%             errors(~index) = Inf;
%             [minE,ind] = min(errors);
%             
%             if widthErrors(i) < minE
%                 errorTable( ind ) = widthErrors(i);
%                 widthsTable( ind ) = widthPatterns(i);
%             end
%         end
%     % Else
%     else
%         % Look up this width in T. If empty, recurse.
%         index = indexTable == upToWidth;
%         % If there's no entry in errorTable for this width
%         if sum( index ) == 0
%             % We need to recurse and find the error
%             [upToError,widths] = dynamicCamAssignment(numCameras,n,upToWidth,positions,shotLengths);
%         else
%             errors = errorTable;
%             errors(~index ) = Inf;
%             [~,ind] = min(errors);
%             upToError = errorTable( ind );
%             widths = widthsTable( ind );
%         end
%         
%         % The combined error for this lensWidth is equal to the
%         % upToError plus the lens error placed at the end.
%         lensError = calcCost(upToWidth, positions(1:size(upToWidth),:),shotLengths(1:size(upToWidth)));
%         
%         % Set error and width pattern that achieves this error.
%         widthErrors(i) = upToError + lensError;
%         widthPatterns(i) = {[widths{:} curCam]};
%         
%         % Add this entry to the index table ONLY if it hasn't been
%         % added before OR if it is the minimum error
%         % (and if so, replace the existing minimum error)
%         newIndex = upToWidth+camWidth;
%         index = indexTable == newIndex;
%         if sum( index ) == 0
%             % Doesn't exist in the table; add
%             indexTable = [indexTable newIndex];
%             errorTable = [errorTable widthErrors(i)];
%             widthsTable = [widthsTable widthPatterns(i)];
%         else
%             % Replace existing value if error is less.
%             errors = errorTable;
%             errors( ~index ) = Inf;
%             [minE,ind] = min(errors);
%             
%             if widthErrors(i) < minE
%                 errorTable( ind ) = widthErrors(i);
%                 widthsTable( ind ) = widthPatterns(i);
%             end
%         end
%     end
% end
% 
% % Find minimum error and return best pattern of widths.
% [minError,index] = min( widthErrors );
% cameraConfig = widthPatterns(index);
% end
