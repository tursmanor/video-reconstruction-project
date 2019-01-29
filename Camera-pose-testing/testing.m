% testing
clearvars; close all;

makeLine =@(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

%% Problem 1: just focal length inside constr, no vertical lines
camPos = [3 3.5 4; 5.5 5 4.5];
camF = [4.5 6.5];
constrP = [4 5 6; 4 4 4];
constrF = [5 5];
constraint = [constrP(:,3)'; constrF; constrP(:,1)'];

if (isBadPosition([camPos camF'], constrP, constrF) == 1)
    disp('pass');
else
    disp('fail');
end

[curOpt,msg,posOut] = pathOpt2D([1 9],[10 9],constraint);

figure;
%[~] = drawCamera(camPos,makeLine,[0 10 0 10],camF,'red'); hold on;
[~] = drawCamera(constrP,makeLine,[0 10 0 10],constrF,'blue'); hold on;
axis([0 10 0 10]);
plot(posOut(:,1),posOut(:,2));

%% Problem 2: just focal length inside constr, no vertical lines
% camPos = [4 3 2; 1 1 1];
% camF = [3 3.5];
% constrP = [6 6 6; 4 5 6];
% constrF = [5 5];
% 
% figure;
% [~] = drawCamera(camPos,makeLine,[0 10 0 10],camF,'red'); hold on;
% [~] = drawCamera(constrP,makeLine,[0 10 0 10],constrF,'blue'); hold on;
% axis([0 10 0 10]);
% 
% if (isBadPosition([camPos camF'], constrP, constrF) == 1)
%     disp('pass');
% else
%     disp('fail');
% end

%% Problem 3: just focal length inside, vertical lines
% camPos = [4 3 2; 4 4 4];
% camF = [3 6];
% constrP = [5 5.5 6; 4 4.5 5];
% constrF = [5 5];
% 
% figure;
% [~] = drawCamera(camPos,makeLine,[0 10 0 10],camF,'red'); hold on;
% [~] = drawCamera(constrP,makeLine,[0 10 0 10],constrF,'blue'); hold on;
% axis([0 10 0 10]);
% 
% if (isBadPosition([camPos camF'], constrP, constrF) == 1)
%     disp('pass');
% else
%     disp('fail');
% end

%% Problem 4: a pass case
% camPos = [4 3 2; 1 1 1];
% camF = [3 2];
% constrP = [6 6 6; 4 5 6];
% constrF = [5 5];
% 
% figure;
% [~] = drawCamera(camPos,makeLine,[0 10 0 10],camF,'red'); hold on;
% [~] = drawCamera(constrP,makeLine,[0 10 0 10],constrF,'blue'); hold on;
% axis([0 10 0 10]);
% 
% if (isBadPosition([camPos camF'], constrP, constrF) == 0)
%     disp('pass');
% else
%     disp('fail');
% end

%% Functions to test
function [out] = isBadPosition(curPos, constrP, constrF)
% check if the current position is within the constraint
% if so, returns true, otherwise returns false
% assumes curPos is a 2x4 matrix where the first three entries are the
% position and the fourth is the focal length

constraints = [constrP(:,3)'; constrF; constrP(:,1)'];
[pt,slopeL,slopeR,~,~] = makeLines(constraints);

if (slopeL > 0), slopeL = slopeL * -1; end
if (slopeR < 0), slopeR = slopeR * -1; end

c = [curPos(2,:) + (slopeL * (curPos(1,:) - pt(1))) - pt(2);   % left line
     curPos(2,:) - (slopeR * (curPos(1,:) - pt(1))) - pt(2)];   % right line

% check if near vertical lines-- only have to deal with
% vertical instead of vertical and horizontal since the fov
% lines will be perpendicular
if (abs(slopeL) > 10)
    % both x and y components need to be greater than f
    locations = sum(curPos > constrF');
    
    % at least one point is in a bad location
    if sum(locations == 2) > 0
        c(1,:) = [1 1 1 1];
    end    
elseif (abs(slopeR) > 10)
    % x must be smaller than fx and y must be larger than fy
    locationsX = curPos(1,:) < constrF(1);
    locationsY = curPos(2,:) > constrF(2);
    
    % at least one point is in a bad location
    if sum((locationsX + locationsY) == 2) > 0
        c(2,:) = [1 1 1 1];
    end
end

% pos is bad if there isn't at least one negative value in each (x,y) point
if (sum((c(1,:) <= 0) | (c(2,:) <= 0)) >= 4)
    out = 0;
else
    out = 1;
end

end