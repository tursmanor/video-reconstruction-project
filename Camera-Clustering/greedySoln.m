%% Greedy setup for clustering
clearvars; close all;

load datasetv1.mat

n = size(dataset,2);   
out = zeros(1,n);
sceneSize = [0 10 0 10];

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
    [opt,bestCam] = trajOptOutput(prevCam,dataset,cams,ind,i);
    %[opt,bestCam] = lineOutput(prevCam,dataset,cams,ind,i,sceneSize);
    
    % check versus some time threshold to determine whether or not to
    % assign shot to an existing camera or a new camera
    thresh = (dataset(i-1).frame / 30);
    if (opt <= thresh)
        if (bestCam == 0), disp('BAD'),end
        out(i) = bestCam;
        ind(bestCam) = i;
    else
        cams = [cams cams(end)+1];
        ind = [ind i];
        out(i) = cams(end);   
    end
end

% make full dataset with new output labeling
algoOut = dataset;
for i=1:n
    algoOut(i).gtCam = out(i);
end
dataset = algoOut;

save('greedyV1Traj','dataset');

%% Helpers

% trajectory optimization
function [opt,bestCam] = trajOptOutput(prevCam,dataset,cams,ind,curShot)
opt = inf;
bestCam = 0;
for j=cams
    
    if (j == prevCam)
        continue;
    else    
        constrPos = dataset(ind(prevCam)).pos;
        constrF = dataset(ind(prevCam)).f;
        constr = [constrPos(:,3)'; constrF; constrPos(:,1)'];
        curPos = dataset(curShot).pos;
        [curOpt,~,pos] = pathOpt2D(dataset(ind(j)).pos(:,2)',curPos(:,2)',constr);           
    end
    
    % calculate aggregate distance travelled, assuming d = vt, w/ v = 1
     time = curOpt(1);
    if (time == inf)
        d = inf;
    else
        d = 0;
        for i=1:(size(pos,1)-1)
            d = d + norm(pos(i+1,:) - pos(i,:));
        end
    end
    
    if (d < opt)
        opt = d;
        bestCam = j;
    end
end
end

% straight line distance
function [opt,bestCam] = lineOutput(prevCam,dataset,cams,ind,curShot,sceneSize)

opt = inf;
bestCam = 0;
for j=cams
    
    if (j == prevCam)
        continue;
    else
        
        startPos = dataset(ind(j)).pos(:,2)';
        goalPos = dataset(curShot).pos(:,2)';
        unitVec = (goalPos - startPos) / norm(goalPos - startPos);
        
        curTime = dataset(curShot-1).frame / 30;
        
        % get current constraint as a polygon
        constr = makeConstraint(dataset(curShot-1).pos,dataset(curShot-1).f,sceneSize);
            
        % get endpoint of straight line path for current shot
        tmpEnd = startPos + (curTime * unitVec);

        % does it intersect a constraint
        [x,y] = polyxpoly([startPos(1) tmpEnd(1)],[startPos(2) tmpEnd(2)],constr(:,1),constr(:,2));
        intersection = [x y];
        
        if (isempty(intersection))
            % cap at end goal
            if (norm(tmpEnd - startPos) > norm(goalPos - startPos))
                tmpEnd = goalPos;
                d = norm(goalPos - startPos);
                disp('Made it to goal');
            else
                d = inf;
            end
        else
            disp('Intersected constraint');
            d = inf;
        end
    end
    
    if (d < opt) && (isequal(tmpEnd,goalPos))
        opt = d;
        bestCam = j;
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
