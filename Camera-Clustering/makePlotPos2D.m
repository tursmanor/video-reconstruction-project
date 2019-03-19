function [plotPos2D] = makePlotPos2D(params)
gridN = params.gridN;
maxPosx = params.maxPosx;
minPosx = params.minPosx;
maxPosy = params.maxPosy;
minPosy = params.minPosy;

plotPos2D = @a;

    function stop = a(x,optimValues,state,varargin)
        % edited optimplotx to pull out position info from x
        
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
                    plot(posX,posY); hold on;
                    ylabel('pos y'); hold on;
                    xlabel('pos x'); hold on;
                    %ylim(sort([minPosy maxPosy])); hold on;
                    %xlim(sort([minPosx maxPosx])); hold on;
                    axis([0 10 0 10]);
                else
                    plot(posX,posY); hold on;
                    pause(0.5);
                end
        end
        
    end
end