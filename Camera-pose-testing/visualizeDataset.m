%% Visualize dataset
close all; clearvars;
load dataset.mat

sceneSize = [0 10 0 10];
makeLine = @(x,x1,y1,x2,y2) ((y2 - y1)/(x2 - x1)) * (x - x1) + y1;

for i = 1:10
    figure(i);
    axis([0 10 0 10]); hold on;
    n = 1;
    for j=1:2:60 % or do j = 1 to look at one
        drawCamera(dataset(i).pos(j:j+1,:),makeLine,sceneSize,dataset(i).f(n,:)'); hold on;
        n = n + 1;
    end
end