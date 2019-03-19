function [range,outLine] = boundsCheck(line,idx,f,sceneSize,makeLine)
% bounds checking for drawing the fov lines

if (line(1,idx) < f(1))
    range = linspace(line(1,idx),sceneSize(2));
    outLine = makeLine(range,line(1,idx),line(2,idx),f(1),f(2));
elseif (line(1,idx) == f(1))
    % check y values
    if (line(2,idx) < f(2))
        outLine = linspace(line(2,idx),sceneSize(4));
    else
        outLine = linspace(sceneSize(3),line(2,idx));
    end
    range = repmat(f(1),size(outLine));
else
    range = linspace(sceneSize(1),line(1,idx));
    outLine = makeLine(range,line(1,idx),line(2,idx),f(1),f(2));
end

end