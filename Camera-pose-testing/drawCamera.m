function [] = drawCamera(line, makeLine, sceneSize,f)

[lRange,lLine] = boundsCheck(line,1,f,sceneSize,makeLine);
[rRange,rLine] = boundsCheck(line,3,f,sceneSize,makeLine);

scatter(line(1,:),line(2,:)); hold on;
scatter(f(1),f(2)); hold on;
plot(lRange, lLine); hold on;
plot(rRange, rLine); hold on;

end