%% Greedy setup for clustering
clearvars; close all;

load Results/dataset-test2.mat

n = size(dataset,2);   
out = struct('cam',0, ...
             'pos',zeros(2,3), ...
             'f',zeros(1,2));

% first two frames are cams 1 and 2
out(1).cam = 1;
[out(1).pos,out(1).f] = avgCamera(dataset(1).pos,dataset(1).f);

out(2).cam = 2;
[out(2).pos,out(2).f] = avgCamera(dataset(2).pos,dataset(2).f);

cams = [1,2];   % active cameras in scene
ind = [1,2];    % indices where previous cameras were found (camera 1 was
                % last at index 1 of out, etc.)

%% Cluster assignments
for i=3:n
    
    prevCam = out(i-1).cam;
    
    % get average position and focal length for camera
    curData = dataset(i);
    [avgP,avgF] = avgCamera(curData.pos,curData.f);
    
    % move all current cameras to new position 
    %[opt,msg,bestCam] = trajOptOutput(prevCam,out,cams,avgP,ind);
    [opt,bestCam] = lineOutput(prevCam,out,cams,avgP,ind,out(i-1).f);
    
    % check versus some time threshold to determine whether or not to
    % assign shot to an existing camera or a new camera
    thresh = (dataset(i-1).frame / 30);
    if (opt(1) <= thresh)
        if (bestCam == 0), disp('BAD'),end
        out(i).cam = bestCam;
        ind(bestCam) = i;
    else
        cams = [cams cams(end)+1];
        ind = [ind i];
        out(i).cam = cams(end);   
    end
    
    out(i).pos = avgP;
    out(i).f = avgF;
    
end

%save('clustering-radius-test','out');

%% Quantify results
algoOut = vertcat(out.cam);
gtOut = vertcat(dataset.gtCam); 
accuracy = sum(algoOut == gtOut) / n

%% Helpers

% trajectory optimization
function [opt,msg,bestCam] = trajOptOutput(prevCam,out,cams,avgP,ind)

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
function [opt,bestCam] = lineOutput(prevCam,out,cams,avgP,ind,f)

opt = inf;
bestCam = 0;
for j=cams
    
    if (j == prevCam)
        continue;
    else
        
        conPos = out(ind(prevCam)).pos;
        conPos = [conPos(:,1)' conPos(:,2)' conPos(:,3)'];
        curPos = [avgP(:,1)' avgP(:,2)' avgP(:,3)'];
        prevPos = out(ind(j)).pos;
        prevPos = [prevPos(:,1)' prevPos(:,2)' prevPos(:,3)'];
        
        % set up line intersection as a system of linear equations
        aVal = @(x1,x2,y1,y2) -(x2 - x1)/(y2 - y1);
        bVal = @(x1,x2,y1,y2) (-y1 * ((x2-x1)/(y2-y1))) + x1;
        slope = @(x1,x2,y1,y2) (y2 - y1)/(x2 - x1);
        
        aValPath = aVal(prevPos(3),curPos(3),prevPos(4),curPos(4));
        bValPath = bVal(prevPos(3),curPos(3),prevPos(4),curPos(4));
        pathSlope = slope(prevPos(3),curPos(3),prevPos(4),curPos(4));
        
        aValLeft = aVal(conPos(5),f(1),conPos(6),f(2));
        bValLeft = bVal(conPos(5),f(1),conPos(6),f(2));
        aValRight = aVal(conPos(1),f(1),conPos(2),f(2));
        bValRight = bVal(conPos(1),f(1),conPos(2),f(2));
        
        ALeft =  [1 aValLeft; 1 aValPath];
        ARight = [1 aValRight; 1 aValPath];
        bLeft =  [bValLeft; bValPath];
        bRight = [bValRight; bValPath];
        
        leftIntersect = ALeft\bLeft;
        rightIntersect = ARight\bRight;
        
        % check if intersection occurs
        if (~testIntersectionBounds(leftIntersect,pathSlope,f,curPos(3:4),prevPos(3:4)) || ...
            ~testIntersectionBounds(rightIntersect,pathSlope,f,curPos(3:4),prevPos(3:4)))
            continue;
        else
            % using d/v = t, where v is one m/s
            d = norm(prevPos(3:4) - curPos(3:4));
        end
        
    end
        
    if (d < opt)
        opt = d;
        bestCam = j;
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
