%% Rapidly expanding random tree for path planning
close all; clearvars;

%% Testing
% Scenario 1: from (1,1) to (9,9) with no constraint
% Scenario 2: from (1,1) to (9,9) with one FOV constraint blocking middle
% Scenario 3: from (1,1) to (9,9) with one FOV constraint blocking side
goal = [9 9];
sceneSize = [0 10 0 10];
constraint = [3 3; 1 6; 5 4];

figure;
axis(sceneSize); hold on;
fill(constraint(:,1),constraint(:,2),'b'); hold on;
G = myRRT([1 2], 100, 0.1, goal, sceneSize, constraint);

% output path will be the shortest path from start to goal

%% Function
% qInit is an x,y position in space
function [G] = myRRT(qInit,K,qDelta,goal,sceneSize,constraint)
% using pseudocode from https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
G = graph();
G = addnode(G,num2str(qInit));

scatter([qInit(1) goal(1)],[qInit(2) goal(2)],'r'); hold on;

for k=1:K
    
    qRand = getRandConf(goal,sceneSize,constraint);
    scatter(qRand(1),qRand(2),'x'); hold on;
    
    qNear = nearestVertex(qRand,G);
    qNew = newConf(qNear,qRand,qDelta,constraint);
    
    if(~isempty(qNew))  
        G = addnode(G,num2str(qNew));
        scatter(qNew(1),qNew(2),'b'); hold on;
        
        G = addedge(G,findnode(G,num2str(qNear)),findnode(G,num2str(qNew)));
        plot([qNew(1) qNear(1)],[qNew(2) qNear(2)]); hold on;
    end
end
end

% returns a random point in the scene area that is not in the constraint
% constraint- mx2 matrix of the vertices defining a polygon 
function [q] = getRandConf(goal,sceneSize,constraint)

x1 = sceneSize(1);
x2 = sceneSize(2);
y1 = sceneSize(3);
y2 = sceneSize(4);

goodPt = false;

while (goodPt == false)
    
    % skew distribution so 10% of the time, we go to the goal position
    x = rand();
    if(x >= 0.9)
        q = goal;
    else
        q = [(x1+(x1+x2)*rand(1,1)) (y1+(y1+y2)*rand(1,1))];
    end
    
    % test if q is in the constraint
    if (isempty(constraint))
        goodPt = true;
    else
        in = inpolygon(q(1),q(2),constraint(:,1),constraint(:,2));
        if(~in), goodPt = true; end
    end
end
end

function [closestQ] = nearestVertex(q,G)

    nodes = G.Nodes;
    d = inf;
    
    for i=1:size(nodes,1)
        
        curNode = nodes{i,'Name'};
        curPos = str2num(cell2mat(curNode));
        
        curDist = norm(q - curPos);
        if curDist < d
            d = curDist;
            closestQ = curPos;
        end 
    end
end

function [q] = newConf(qNear,qRand,qDelta,constraint)

    q = qNear + qDelta*(qRand - qNear);

    % if the path ends inside the constraint, reject the path
    if (~isempty(constraint))
        in = inpolygon(q(1),q(2),constraint(:,1),constraint(:,2));
        if(in)
            q = [];
        end
    end
end