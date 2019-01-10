function [figHandles] = drawCamera(line,makeLine,sceneSize,f,color)
% draw 1D camera and its fov

[lRange,lLine] = boundsCheck(line,1,f,sceneSize,makeLine);
[rRange,rLine] = boundsCheck(line,3,f,sceneSize,makeLine);

a = scatter(line(1,:),line(2,:),[],color); hold on;
b = scatter(f(1),f(2),[],color); hold on;
c = plot(lRange, lLine,'Color',color); hold on;
d = plot(rRange, rLine,'Color',color); hold on;

% fill area between lines
xL = lRange(lRange >= f(1));
yL = lLine(lRange >= f(1));
xR = rRange(rRange <= f(1));
yR = rLine(rRange <= f(1));

e = fill([xR,xL],[yR,yL],color,'FaceAlpha',0.3); hold on;
set(e,'Visible','off');

figHandles = [a,b,c,d,e];
end