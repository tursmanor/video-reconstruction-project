% expand each edge point of the previous polygon to make a new, bigger, and
% better polygon
function [output]= expandPolygon(curPolygon,radius,constraint,sceneSize)
global nope;
digits(10);

allPts = [];
makeLine =@(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

% draw a new circle around each existing polygon pt
for i = 1:size(curPolygon,1)
    startPt = curPolygon(i,:);
    pts = samplePoints(startPt,radius);
    allPts = [allPts; pts];
end

boundInd = boundary(allPts(:,1),allPts(:,2),0.1);
newPolygon = allPts(boundInd,:);

% bounds clamping
newPolygon(newPolygon(:,1) < sceneSize(1),1) = sceneSize(1);
newPolygon(newPolygon(:,1) > sceneSize(2),1) = sceneSize(2);
newPolygon(newPolygon(:,2) < sceneSize(3),2) = sceneSize(3);
newPolygon(newPolygon(:,2) > sceneSize(4),2) = sceneSize(4);

% take intersection with constraint
[x,y] = polyxpoly(newPolygon(:,1),newPolygon(:,2),constraint(:,1),constraint(:,2));
intersection = [x y];
tmpInt = intersection;
numInt = size(intersection,1);

if (isempty(intersection) || numInt == 1)
    output = newPolygon;
elseif (numInt > 2)
    disp('nope, more than 2 intersections')
    nope = 1;
    output = newPolygon;
else
    
    % three nonempty intersection cases: +- sloped line cut (results in 1 new
    % polygon), v cut (resulting in 2 new polygons)
    % pick vertex, check if it's in the constraint. if it is, pick new vertex
    for i=1:size(newPolygon,1)
        startPt = newPolygon(i,:);
        startInd = i;
        in = inpolygon(startPt(1),startPt(2),constraint(:,1),constraint(:,2));
        if (~in), break; end
    end
    
    % cycle through vertices, and remove those that fall between the constraint
    % intersections-- this is the scenario where there are only two
    % intersection points -- need another case for 4 intersection points
    curPt = startPt;
    curInd = startInd;
    newPolygon2 = curPt;
    inConstraint = 0;
    addedFOVPt = 0;
    addedLine = 0;
    for i=1:size(newPolygon,1)
        
        curInd = mod(curInd + 1,size(newPolygon,1));
        if (curInd == 0), curInd = size(newPolygon,1); end
        
        nextPt = newPolygon(curInd,:);
        
        % check if an intersection point lies on the line segment between
        % the current and next point. if it does, remove if from the list
        % of intersection points
        if(numInt > 0)
            for j=1:numInt
                approxEqual = 0;
                curIntPt = intersection(j,:);
                y = makeLine(curIntPt(1),curPt(1),curPt(2),nextPt(1),nextPt(2));
                
                if (round(y,4) == round(curIntPt(2),4))     
                    maxPtX = max(curPt(1),nextPt(1));
                    minPtX = min(curPt(1),nextPt(1));
                    maxPtY = max(curPt(2),nextPt(2));
                    minPtY = min(curPt(2),nextPt(2));
                    
                    approxEqual = ((curIntPt(1) <= maxPtX) && ...
                                   (curIntPt(1) >= minPtX) && ...
                                   (curIntPt(2) <= maxPtY) && ...
                                   (curIntPt(2) >= minPtY));
                end
                
                curIntPt = round(curIntPt,2);
                
                % vertical line check
                if ((nextPt(1) == curPt(1)) && (curIntPt(1) == curPt(1)))
                    maxPt = max(curPt(2),nextPt(2));
                    minPt = min(curPt(2),nextPt(2));
                    if ((curIntPt(2) <= maxPt) && (curIntPt(2) >= minPt))
                        approxEqual = 1;
                    else
                        approxEqual = 0;
                    end
                end
                
                % horizontal line check
                if ((nextPt(2) == curPt(2)) && (curIntPt(2) == curPt(2)))
                    maxPt = max(curPt(1),nextPt(1));
                    minPt = min(curPt(1),nextPt(1));
                    if ((curIntPt(1) <= maxPt) && (curIntPt(1) >= minPt))
                        approxEqual = 1;
                    else
                        approxEqual = 0;
                    end
                end
                
                if (approxEqual)
                    inConstraint = inConstraint + 1;
                    newPolygon2 = [newPolygon2; curIntPt];
                    intersection(j,:) = [];
                    numInt = numInt - 1; 
                    break;
                end
            end
        end
        
        % if so, nextPt is not added to the new polygon, and we throw up a flag
        if (inConstraint == 0)
            newPolygon2 = [newPolygon2; nextPt];
            
        elseif (inConstraint == 1)
            % add the tip of the FOV if it's inside the current polygon
            in = inpolygon(constraint(1,1),constraint(1,2),newPolygon(:,1),newPolygon(:,2));
            lastPt = newPolygon2(end,:);
            
            if(in && ~addedFOVPt)
                newPolygon2 = [newPolygon2; populateLine(makeLine,lastPt,constraint(1,:))];
                addedFOVPt = 1;
            end
            
            if (~addedLine)
                if(in)
                    lastPt = constraint(1,:);
                end
                newPolygon2 = [newPolygon2; populateLine(makeLine,lastPt,intersection)];
                addedLine = 1;
            end
            
        elseif (inConstraint == 2)
            % turn off the flag after we've passed the constraint a second time
            inConstraint = 0;
            %curInd = curInd + 1;
        end
        
        curPt = nextPt;
    end
    
    output = newPolygon2;  
end

% figure(3);
% clf;
% fill(output(:,1),output(:,2),'r','FaceAlpha',0.2); hold on;
% axis([0 10 0 10]); hold on;
% scatter(curPolygon(:,1),curPolygon(:,2)); hold on;
% scatter(output(:,1),output(:,2)); hold on;
% fill(constraint(:,1),constraint(:,2),'b','FaceAlpha',0.2);
% disp('eh');

end

% sample points every theta degrees of a circle centered at [center]  with
% radius [radius] to create a polygon
function [pts] = samplePoints(center, radius)

theta = pi/16;
pts = [];
curAngle = 0;
while (curAngle < (2*pi))
    
    x = radius*cos(curAngle)+center(1);
    y = radius*sin(curAngle)+center(2);
    
    pts = [pts; x y];
    curAngle = curAngle + theta;
    
end

end

% populate line with points
function [pts] = populateLine(makeLine,stPt,endPt)

pts = [];
xSamples = linspace(stPt(1),endPt(1),10);

% vertical line check
if (stPt(1) == endPt(1))
    pts = [repmat(stPt(1),[10,1]) linspace(stPt(2),endPt(2),10)'];
    % horizontal line check
elseif (stPt(2) == endPt(2))
    pts = [xSamples' repmat(stPt(2),[10,1])];
else
    for x=xSamples
        y = makeLine(x,stPt(1),stPt(2),endPt(1),endPt(2));
        pts = [pts; x y];
    end
end

end