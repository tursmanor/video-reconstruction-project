function stop = plotPos(x,optimValues,state,varargin)
% edited optimplotx to pull out position info from x

global gridN;
global minPos;
global maxPos;

stop = false;
switch state
    case 'iter'
        % Reshape if x is a matrix
        x = x(:);
        
        % extract position from x
        pos = x(2 : 1 + gridN);
        
        % get time steps
        sim_time = x(1);
        delta_time = sim_time / gridN;
        times = 0 : delta_time : sim_time - delta_time;
        
        if optimValues.iteration == 0
            figure(1);
            plot(times,pos); hold on;
            ylabel('Position'); hold on;
            xlabel('Time'); hold on;
            ylim([minPos maxPos]); hold on; 
        else
            plot(times,pos); hold on;
            pause(0.5);
        end
end

