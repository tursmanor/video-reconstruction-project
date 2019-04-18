function [fovHan] = compactVisualization(dataset,shotNum)
% compactly visualize a sequence

n = size(dataset,2);
cameraSeq = vertcat(dataset.gtCam);
positions = zeros(2,n);
axis([0 10 0 10]); hold on;

% draw each position
for i=1:n
    [avgP,~] = avgCamera(dataset(i).pos,dataset(i).f);
    curPos = avgP(:,2);
    positions(:,i) = curPos;
end

if (shotNum == 1)
    drawScatter(dataset,n,'k')
    fovHan = 0;
else
    % draw path for shotNum in shot sequence
    curCam = cameraSeq(shotNum);
    prevCam = cameraSeq(shotNum-1);
    
    % draw constraint
    [avgP,avgF] = avgCamera(dataset(shotNum-1).pos,dataset(shotNum-1).f);
    fovHan = drawCameraFOV(avgP,avgF,getColor(prevCam));
    scatter(positions(1,shotNum-1),positions(2,shotNum-1),'filled',getColor(prevCam));
    hold on;
    
    ind = lastAppearance(shotNum,cameraSeq);
    
    if (ind ~= 0)
        pt2 = positions(:,shotNum);
        pt1 = positions(:,ind);
        diff = pt2 - pt1;
        quiver(pt1(1),pt1(2),diff(1),diff(2),'color',getColor(curCam),'MaxHeadSize',2/norm(pt2-pt1));
        hold on;
        scatter([pt1(1) pt2(1)],[pt1(2) pt2(2)],'filled',getColor(curCam));
        hold on;
    else
        pt2 = positions(:,shotNum);
        scatter(pt2(1),pt2(2),'filled',getColor(curCam));
        hold on;
    end
    
end

end

function [] = drawScatter(dataset,n,color)

    for i=1:n
        [avgP,~] = avgCamera(dataset(i).pos,dataset(i).f);
        curPos = avgP(:,2);
        scatter(curPos(1),curPos(2),'filled',color);
        hold on;    
    end

end

function c = getColor(camId)

if (camId == 1)
    c = 'blue';
elseif (camId == 2)
    c = 'red';
elseif (camId == 3)
    c = 'green';
else
    c = 'magenta';
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

function [figHandles] = drawCameraFOV(avgP,f,color)
% draw 1D camera's fov

makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;
sceneSize = [-10 15 -10 15];

[lRange,lLine] = boundsCheck(avgP,1,f,sceneSize,makeLine);
[rRange,rLine] = boundsCheck(avgP,3,f,sceneSize,makeLine);

% fill area between lines
xL = lRange(lRange >= f(1));
yL = lLine(lRange >= f(1));
xR = rRange(rRange <= f(1));
yR = rLine(rRange <= f(1));

e = fill([xR,xL],[yR,yL],color,'FaceAlpha',0.3); hold on;

figHandles = e;
end
