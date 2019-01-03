function stop = plotPos2D(x,optimValues,state,varargin)
% edited optimplotx to pull out position info from x

global gridN
global maxPosx
global minPosx
global maxPosy
global minPosy

stop = false;
switch state
    case 'iter'
        % Reshape if x is a matrix
        x = x(:);
        
        % extract position from x
        posX = x(2 : 1 + gridN);
        posY = x(2 + gridN : 1 + gridN * 2);
        
        % get time steps
        sim_time = x(1);
        delta_time = sim_time / gridN;
        times = 0 : delta_time : sim_time - delta_time;
        
        if optimValues.iteration == 0
            figure(1);
            plot(posY,posX); hold on;
            ylabel('pos y'); hold on;
            xlabel('pos x'); hold on;
            ylim(sort([minPosy maxPosy])); hold on;
            xlim(sort([minPosx maxPosx])); hold on;
        else
            plot(posY,posX); hold on;
            %pause(0.5);
        end
end

