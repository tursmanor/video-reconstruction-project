%% Rapidly expanding random tree for path planning
% close all; clearvars;

%% Testing
% Scenario 1: from (1,1) to (9,9) with no constraint
% Scenario 2: from (1,1) to (9,9) with one FOV constraint blocking middle
% Scenario 3: from (1,1) to (9,9) with one FOV constraint blocking side
% goal = [9 9];
%constraint = [3 3; 1 6; 5 4];
% sceneSize = [0 10 0 10];
% G = graph();
% G = addnode(G,num2str([5 5]));
% G = RRT2(G,100,0.5,sceneSize,[],2,[5 5]);
% A = RRT2(G,100,0.5,sceneSize,[],6,[5 5]);

%% Function
% takes a graph as init
function [G] = RRT(G,K,qDelta,sceneSize,constraint,maxPath,goal)
% using pseudocode from https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree

%figure(2);
%axis(sceneSize); hold on;

for k=1:K
    
    qRand = getRandConf(goal,sceneSize,constraint);   
    qNear = nearestVertex(qRand,G);
    qNew = newConf(qNear,qRand,qDelta,constraint); 
    %scatter(qRand(1),qRand(2),'x'); hold on;
    
    if(~isempty(qNew))     
        % accumulate distance traveled as edge weights
        dist = norm(qNew - qNear);
        [~,ind2] = findedge(G);
        nearNodeID = findnode(G,num2str(qNear));
        newInd = find(ind2 == nearNodeID);
        
        if (isempty(newInd))
            totalDist = dist;
        else
            totalDist = G.Edges.Weight(newInd) + dist;
        end
        
        % check if we can hit the goal in time
        if (isequal(qRand,goal))
            distGoal = norm(goal - qNear); 
            
            if (isempty(newInd))
                tmpDist = distGoal;
            else
                tmpDist = G.Edges.Weight(newInd) + distGoal;
            end

            if (tmpDist <= maxPath)
                totalDist = tmpDist;
                qNew = qRand;
            end 
        end
          
        % check if total distance for path is allowed
        if (totalDist <= maxPath)
            
            if(findnode(G,num2str(qNew)) == 0)
                G = addnode(G,num2str(qNew));
            end
            
            % add edge if it doesn't already exist
            nearNode = findnode(G,num2str(qNear));
            newNode = findnode(G,num2str(qNew));
            if (findedge(G,nearNode,newNode) == 0)
                G = addedge(G,nearNode,newNode,min(totalDist));
            end
            
            %scatter(qNew(1),qNew(2),'b'); hold on;
            %plot([qNew(1) qNear(1)],[qNew(2) qNear(2)]); hold on;
        end
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
