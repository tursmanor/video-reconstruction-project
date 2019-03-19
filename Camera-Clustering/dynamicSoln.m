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
positions = zeros(n,2);
shotLengths = zeros(n,1);
fs = zeros(n,2);
numCam = 4;

for i=1:n
    [avgP,avgF] = avgCamera(dataset(i).pos,dataset(i).f);
    positions(i,:) = avgP(:,2);
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
            break;
        end
        
        constraintRegion = ;
        
        % check if cur camera has appeared before, if so, see if its path
        % intersects the constraint region
        
        
   
        prevCam = curCam;
    end
end

function [out] = lastAppearance();

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
function [minError,cameraConfig] = dynamicCamAssignment(numCameras,n,shotSubset,positions,shotLengths,fs)

global indexTable errorTable widthsTable;

% Collect an error for each camera num
widthErrors = zeros(numCameras, 1);
widthPatterns = cell(numCameras, 1);

for i=1:numCameras
    
    camWidth = 1;
    curCam = i;
    
    % First, get remaining subset width after placing
    % this lens at right-most part of epiSlice
    upToWidth = shotSubset - camWidth;
    
    % If this width is negative, i.e., if the lens width is larger,
    % then this lens cannot fit and we assign a large error
    if upToWidth < 0
        widthErrors(i) = Inf;
        widthPatterns(i) = {-1};
        % If the lens width equals the remaining subset width
    elseif upToWidth == 0
        % This is a base case.
        % Compute the error directly.
        %
        widthErrors(i) = calcCost(upToWidth, positions(1:size(upToWidth),:),shotLengths(1:size(upToWidth)));
        widthPatterns(i) = {curCam};
        
        % It is possible for a length of 'lensWidth' to already
        % exist from a deeper level plus a lens (e.g., 10 will
        % already exist from 5 + 5.
        index = indexTable == camWidth;
        if sum( index ) == 0
            % Doesn't exist in the table; add
            indexTable = [indexTable camWidth];
            errorTable = [errorTable widthErrors(i)];
            widthsTable = [widthsTable widthPatterns(i)];
        else
            % Replace existing value if error is less.
            errors = errorTable;
            errors(~index) = Inf;
            [minE,ind] = min(errors);
            
            if widthErrors(i) < minE
                errorTable( ind ) = widthErrors(i);
                widthsTable( ind ) = widthPatterns(i);
            end
        end
    % Else
    else
        % Look up this width in T. If empty, recurse.
        index = indexTable == upToWidth;
        % If there's no entry in errorTable for this width
        if sum( index ) == 0
            % We need to recurse and find the error
            [upToError,widths] = dynamicCamAssignment(numCameras,n,upToWidth,positions,shotLengths);
        else
            errors = errorTable;
            errors(~index ) = Inf;
            [~,ind] = min(errors);
            upToError = errorTable( ind );
            widths = widthsTable( ind );
        end
        
        % The combined error for this lensWidth is equal to the
        % upToError plus the lens error placed at the end.
        lensError = calcCost(upToWidth, positions(1:size(upToWidth),:),shotLengths(1:size(upToWidth)));
        
        % Set error and width pattern that achieves this error.
        widthErrors(i) = upToError + lensError;
        widthPatterns(i) = {[widths{:} curCam]};
        
        % Add this entry to the index table ONLY if it hasn't been
        % added before OR if it is the minimum error
        % (and if so, replace the existing minimum error)
        newIndex = upToWidth+camWidth;
        index = indexTable == newIndex;
        if sum( index ) == 0
            % Doesn't exist in the table; add
            indexTable = [indexTable newIndex];
            errorTable = [errorTable widthErrors(i)];
            widthsTable = [widthsTable widthPatterns(i)];
        else
            % Replace existing value if error is less.
            errors = errorTable;
            errors( ~index ) = Inf;
            [minE,ind] = min(errors);
            
            if widthErrors(i) < minE
                errorTable( ind ) = widthErrors(i);
                widthsTable( ind ) = widthPatterns(i);
            end
        end
    end
end

% Find minimum error and return best pattern of widths.
[minError,index] = min( widthErrors );
cameraConfig = widthPatterns(index);
end
