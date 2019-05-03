%% Greedy clustering for V2 dataset
close all; clearvars;

%% Setup
load datasetv2.mat

n = size(dataset,2);
out = zeros(1,n);
sceneSize = [0 10 0 10];
badOutput = 0;

% first two frames are cams 1 and 2
out(1) = 1;
out(2) = 2;

cams = [1,2];   % active cameras in scene
ind = [1,2];    % indices where previous cameras were found (camera 1 was
% last at index 1 of out, etc.)

%% Cluster assignments
for i=3:n
    
    prevCam = out(i-1);
    
    % move all current cameras to new position
    %[opt,msg,bestCam] = piecewiseLineOutput(prevCam,out,cams,avgP,ind);
    [opt,bestCam] = lineOutput(prevCam,cams,ind,dataset,i,sceneSize);
        
    % check versus some time threshold to determine whether or not to
    % assign shot to an existing camera or a new camera
    if (opt(1) ~= inf)
        if (bestCam == 0), disp('BAD'),end
        out(i) = bestCam;
        ind(bestCam) = i;
    else
        cams = [cams cams(end)+1];
        ind = [ind i];
        out(i) = cams(end);
    end
    
    % check that nothing is in the new cam's FOV
    constr = makeConstraint(dataset(i).pos,dataset(i).f,sceneSize);
    in = 0;
    for j=1:size(ind,2)
        if (j == out(i))
            continue;
        end
       curPt = dataset(ind(j)).pos(:,2)';
       in = in + inpolygon(curPt(1),curPt(2),constr(:,1),constr(:,2));       
    end
    
    if (in ~= 0)
       disp('Sequence invalid, camera in new FOV');
       badOutput = 1;
       break;
    end
    
end

% make full dataset with new output labeling
algoOut = dataset;
for i=1:n
    algoOut(i).gtCam = out(i);
end
dataset = algoOut;

if (badOutput == 0)
    save('greedyV2','dataset');
end

%% Helpers
% trajectory optimization
function [opt,msg,bestCam] = piecewiseLineOutput(prevCam,out,cams,avgP,ind)

opt = inf;
bestCam = 0;
for j=cams
    
    if (j == prevCam)
        continue;
    else
        
        constrPos = out(ind(prevCam)).pos;
        constrF = out(ind(prevCam)).f;
        constr = [constrPos(:,3)'; constrF; constrPos(:,1)'];
        [curOpt,msg,~] = pathOpt2D(out(ind(j)).pos(:,2)',avgP(:,2)',constr);
        
    end
    
    if ((curOpt(1) < opt(1)) && strcmp(msg.message(2:6),'Local'))
        opt = curOpt;
        bestCam = j;
    end
end

end

% straight line distance
function [opt,bestCam] = lineOutput(prevCam,cams,ind,dataset,curShot,sceneSize)

opt = inf;
bestCam = 0;

for j=cams
    
    if (j == prevCam)
        continue;
    else
        
        % get number of shots camera j had to move in
        % eg., for [1,2,_], 1 has 3-1-1 = 1 shot to move
        numActiveShots = curShot - ind(j) - 1;
        
        startPos = dataset(ind(j)).pos(:,2)';
        goalPos = dataset(curShot).pos(:,2)';
        unitVec = (goalPos - startPos) / norm(goalPos - startPos);
        
        % accumulate possible straight line path per active shot
        dist = 0;
        curEnd = startPos;
        bestEnd = 0;
        for i=1:numActiveShots
            
            shot = ind(j) + i;
            curTime = dataset(shot).frame / 30;
            
            % get current constraint as a polygon
            constr = makeConstraint(dataset(shot).pos,dataset(shot).f,sceneSize);
            
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
               dist = inf;
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
    end
    
    if (dist < opt) && (isequal(curEnd,goalPos))
        opt = dist;
        bestCam = j;
        bestEnd = curEnd;
    end 
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
    boundY1 = [sceneSize(3) sceneSize(4)+10000];
    boundX2 = [sceneSize(1)-10 sceneSize(1)-10];
    boundY2 = [sceneSize(3) sceneSize(4)+10000];
else
    boundX1 = [sceneSize(1)-10 sceneSize(1)-10];
    boundY1 = [sceneSize(3) sceneSize(4)+10000];
    boundX2 = [sceneSize(2)+10 sceneSize(2)+10];
    boundY2 = [sceneSize(3) sceneSize(4)+10000];
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
