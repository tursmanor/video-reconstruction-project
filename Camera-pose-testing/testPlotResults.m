function [] = testPlotResults(pos,f,path,out,makeLine)

figure;
[~] = drawCamera(pos,makeLine,[0 10 0 10],f,'blue'); 
hold on;
scatter(path(1,:),path(2,:));
hold on;
plot(out(:,1),out(:,2));
axis([0 10 0 10]);

end