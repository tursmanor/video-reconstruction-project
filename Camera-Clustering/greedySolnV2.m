%% Greedy clustering for V2 dataset
close all; clearvars;

%% Setup
load datasetv2-2.mat

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
    [opt,bestCam] = rrtOutput(prevCam,cams,ind,dataset,i,sceneSize);
    %[opt,bestCam] = piecewiseLineOutput(prevCam,cams,ind,dataset,i,sceneSize);
    %[opt,bestCam] = lineOutput(prevCam,cams,ind,dataset,i,sceneSize);
        
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
    save('greedyV2RRT','dataset');
end

%% Helpers
% rrt line distance
function [opt,bestCam] = rrtOutput(prevCam,cams,ind,dataset,curShot,sceneSize)

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
        curArea = startPos;
        
        paths = graph();
        paths = addnode(paths,num2str(startPos));
        maxPathLength = 0;

        figure;
        axis(sceneSize);
        title(strcat('Number of shots to move in: ',num2str(numActiveShots))); hold on;
        scatter(startPos(1),startPos(2),'b'); hold on;
        scatter(goalPos(1),goalPos(2),'g','filled'); hold on;
        
        % increase area size based on number of active shots
        for i=1:numActiveShots
            
            shot = ind(j) + i;
            curTime = dataset(shot).frame / 30;
            maxPathLength = maxPathLength + curTime;
            
            % get current constraint as a polygon
            constr = makeConstraint(dataset(shot).pos,dataset(shot).f,sceneSize); 
            
            % add to path graph
            curArea = expandPolygon(curArea,curTime,constr,sceneSize);   
            
            fill(constr(:,1),constr(:,2),'b','FaceAlpha',0.1); hold on;
            fill(curArea(:,1),curArea(:,2),'r','FaceAlpha',0.1); hold on;
            
            % populate graph via RRT
            paths = RRT(paths,100,0.8,sceneSize,constr,maxPathLength,goalPos);
            
            % don't keep going through active shots if we've already hit
            % the goal
            if (findnode(paths,num2str(goalPos)) ~= 0)
                break;
            end 
        end
    end
    
    % if we reached the goal in time, get shortest path from start to goal
    % of final graph
    goalNodeID = findnode(paths,num2str(goalPos));
    if(goalNodeID ~= 0)
        goalEdges = outedges(paths,goalNodeID);
        d = min(paths.Edges{goalEdges,'Weight'});
    else
        d = inf;
    end

    if (d < opt)
       opt = d;
       bestCam = j;
    end
end

end

% piecewise line distance
function [opt,bestCam] = piecewiseLineOutput(prevCam,cams,ind,dataset,curShot,sceneSize)

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
        curArea = startPos;
        
        paths = graph();
        paths = addnode(paths,num2str(startPos));

        figure;
        axis(sceneSize);
        title(strcat('Number of shots to move in: ',num2str(numActiveShots))); hold on;
        scatter(startPos(1),startPos(2),'b'); hold on;
        scatter(goalPos(1),goalPos(2),'g','filled'); hold on;
        
        % increase area size based on number of active shots
        for i=1:numActiveShots
            
            shot = ind(j) + i;
            curTime = dataset(shot).frame / 30;
            
            % get current constraint as a polygon
            constr = makeConstraint(dataset(shot).pos,dataset(shot).f,sceneSize); 
            
            % add to path graph
            curArea = expandPolygon(curArea,curTime,constr,sceneSize);   
            
            fill(constr(:,1),constr(:,2),'b','FaceAlpha',0.1); hold on;
            fill(curArea(:,1),curArea(:,2),'r','FaceAlpha',0.1); hold on;
            
            % if there is only one node in the graph, spread in a circle,
            % otherwise populate as normal
            curIn = inpolygon(goalPos(1),goalPos(2),curArea(:,1),curArea(:,2));
            if((size(paths.Nodes,1) == 1) && ~curIn)
                paths = initGraph(paths,curArea);
            else
                paths = populateGraph(paths,goalPos,curArea,curTime);
            end
            
            % don't keep going through active shots if we've already hit
            % the goal
            if (curIn)
                break;
            end 
        end
    end
    
    % if we reached the goal in time, get shortest path from start to goal
    % of final graph
    if(findnode(paths,num2str(goalPos)) ~= 0)
    [~,d] = shortestpath(paths,findnode(paths,num2str(startPos)), ...
                         findnode(paths,num2str(goalPos)));
    else
        d = inf;
    end

    if (d < opt)
       opt = d;
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
            
            scatter(curPt(1),curPt(2),'r'); hold on;
            plot([curPt(1) centerNode(1)],[curPt(2) centerNode(2)]); hold on;
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
            
            pathsOut = addedge(pathsOut,findnode(pathsOut,num2str(curNode)), ...
                findnode(pathsOut,num2str(newPt)),dist);
            
            scatter(newPt(1),newPt(2),'r'); hold on;
            plot([newPt(1) curNode(1)],[newPt(2) curNode(2)]); hold on;
        end
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
