%% Visualize a v2 dataset
% draw the four camera positions, color code them, and have them move frame
% by frame, visualize growing areas in subplots concurrently

close all; clearvars;
load 'datasetv2.mat'
sceneSize = [-10 15 -10 15];
makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;
filename = 'Results/datasetv2-2.gif';
viewSize = [-1 11 -1 11];

fig = figure(1);
camHandles1 = zeros(4,5);
prevCam1 = 0;

for i = 1:size(dataset,2)
    
    subplot(2,3,1)
    axis(viewSize); hold on;
    title('Dataset');
    plot([0 0 10 10 0],[0 10 10 0 0],'b'); hold on;
    [time,pCam1,cHand1] = plotDatasetFrame(dataset,i,camHandles1,prevCam1);
    prevCam1 = pCam1;
    camHandles1 = cHand1;
    axis('square');
    
    s2 = subplot(2,3,2);
    axis(viewSize); hold on;
    plot([0 0 10 10 0],[0 10 10 0 0]); hold on;
    title('Camera 1 Area');
    plotArea(dataset,i,1,[0 10 0 10]);
    axis('square');
    
    s3 = subplot(2,3,3);
    axis(viewSize); hold on;
    plot([0 0 10 10 0],[0 10 10 0 0]); hold on;
    title('Camera 2 Area');
    plotArea(dataset,i,2,[0 10 0 10]);
    axis('square');
    
    s4 = subplot(2,3,4);
    axis(viewSize); hold on;
    plot([0 0 10 10 0],[0 10 10 0 0]); hold on;
    title('Camera 3 Area');
    plotArea(dataset,i,3,[0 10 0 10]);
    axis('square');
    
    s5 = subplot(2,3,5);
    axis(viewSize); hold on;
    plot([0 0 10 10 0],[0 10 10 0 0]); hold on;
    title('Camera 4 Area');
    plotArea(dataset,i,4,[0 10 0 10]);
    axis('square');
    
    % save as a gif
    im = frame2im(getframe(fig));
    [A,map] = rgb2ind(im,256);
    if (i == 1)
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',time);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',time);
    end
    
    cla(s2),cla(s3),cla(s4),cla(s5);
end

%% Helpers

% draw dataset normally
function [time,prevCam,camHandles] = plotDatasetFrame(dataset,frame,camHandles,prevCam)

makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;
sceneSize = [-10 15 -10 15];

[avgP,avgF] = avgCamera(dataset(frame).pos,dataset(frame).f);
curCam = dataset(frame).gtCam;

% set distinct colors for each camera, assuming 4 cameras
if (curCam == 1)
    color = 'blue';
elseif (curCam == 2)
    color = 'red';
elseif (curCam == 3)
    color = 'green';
else
    color = 'magenta';
end

% erase camera's previous location
if (camHandles(curCam,:) ~= 0)
    set(camHandles(curCam,:),'Visible','off');
end

% draw camera
[figHandles] = drawCamera(avgP,makeLine,sceneSize,avgF,color);
camHandles(curCam,:) = figHandles;

% fill in fov for previous camera
if prevCam ~= 0
    set(camHandles(prevCam,end),'Visible','on');
    cameras = ones(1,4);
    cameras(prevCam) = 0;
    nonActiveCams = camHandles(logical(cameras),end);
    noZeros = nonActiveCams(nonActiveCams ~= 0);
    set(noZeros,'Visible','off');
else
    set(camHandles(curCam,end),'Visible','on');
end

prevCam = curCam;

% get time of shot
time = dataset(frame).frame / 30;

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

function [] = plotArea(dataset,frame,camId,sceneSize)

color = ['b' 'r' 'g' 'm'];
if(frame == 1)
    prevCam = dataset(1).gtCam;
    constraint = makeConstraint(dataset(1).pos,dataset(1).f,sceneSize);
    fill(constraint(:,1),constraint(:,2),color(prevCam),'FaceAlpha',0.6); hold on;
else
    prevCam = dataset(frame-1).gtCam;
    constraint = makeConstraint(dataset(frame-1).pos,dataset(frame-1).f,sceneSize);
    fill(constraint(:,1),constraint(:,2),color(prevCam),'FaceAlpha',0.6); hold on;

    areas = dataset(frame).A;
    if (~isempty(areas{camId}))
        pts = areas{camId};
        fill(pts(:,1),pts(:,2),color(camId),'FaceAlpha',0.3); hold on;
    end  
end

end