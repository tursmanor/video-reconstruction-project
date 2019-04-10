function [] = visualizeTwoDatasets(dataset1, dataset2, filenameOut)
% side by side, draw two datasets
% assumes datasets have same number of shots
% assumes shot times are also the same across datasets

filename = strcat('Results/',filenameOut,'.gif');

camHandles1 = zeros(4,5);
camHandles2 = zeros(4,5);
prevCam1 = 0;
prevCam2 = 0;
n = size(dataset1,2);

fig = figure(1);

% draw each frame side by side
for i=1:n
    
    subplot(1,2,1);
    axis([0 10 0 10]); hold on;
    title('GT Dataset');
    [t1,pCam1,cHand1] = plotDatasetFrame(dataset1,i,camHandles1,prevCam1);
    prevCam1 = pCam1;
    camHandles1 = cHand1;
    
    subplot(1,2,2);
    axis([0 10 0 10]); hold on;
    title('Dynamic Tree Best Solution');
    [~,pCam2,cHand2] = plotDatasetFrame(dataset2,i,camHandles2,prevCam2);
    prevCam2 = pCam2;
    camHandles2 = cHand2;
    
    % save as a gif
    im = frame2im(getframe(fig));
    [A,map] = rgb2ind(im,256);
    if (i == 1)
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',t1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',t1);
    end
    
end


end

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
