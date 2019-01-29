function [constraints2D] = makeConstraints2D(params)
gridN = params.gridN;
maxPosx = params.maxPosx;
minPosx = params.minPosx;
maxPosy = params.maxPosy;
minPosy = params.minPosy;
maxVelocity = params.maxVelocity;
maxAcceleration = params.maxAcceleration;
constraints = params.constraints;

constraints2D = @a;

    function [c,ceq] = a(x)
        delta = 0;
        
        % Calculate the timestep
        sim_time = x(1);
        delta_time = sim_time / gridN;
        
        % Get the states / inputs out of the vector
        positions(:,1) = x(2 : 1 + gridN);
        positions(:,2) = x(2 + gridN : 1 + gridN * 2);
        vels(:,1) = x(2 + gridN * 2 : 1 + gridN * 3);
        vels(:,2) = x(2 + gridN * 3 : 1 + gridN * 4);
        accs(:,1) = x(2 + gridN * 4 : 1 + gridN * 5);
        accs(:,2) = x(2 + gridN * 5 : end);
        
        %% Constrain component velocities and accelerations
        % for square centered at (5,5), width = 2
        % -max(abs(positions(:,1) - 5),abs(positions(:,2) - 5)) + 2]
        
        % no position constraints
        if (constraints == 0)
            c = [-maxVelocity + sqrt(vels(:,1).^2 + vels(:,2).^2);     % max vel component constraint
                -maxAcceleration + sqrt(accs(:,1).^2 + accs(:,2).^2)]; % max acc component constraint
        else
            % get lines for cam's fov
            [pt,slopeL,slopeR,~,~] = makeLines(constraints);
            %leftSlope = 0;
            %rightSlope = 0;
           
            % cap slope to avoid inf with vertical lines
            if (abs(slopeL) > 10), slopeL = -100; end
            if (abs(slopeR) > 10), slopeR = 100; end
            
            leftLineConst = positions(:,2) - (slopeL * (positions(:,1) - pt(1))) - pt(2);
            rightLineConst =  positions(:,2) - (slopeR * (positions(:,1) - pt(1))) - pt(2);
          
            % we expect the left line to have a neg slope, right line a pos slope
            if (slopeL > 0)
                leftLineConst = leftLineConst * -1; 
                %leftSlope = 1;
            end
            if (slopeR < 0)
                rightLineConst = rightLineConst * -1; 
                %rightSlope = 1;
            end
    
            % OR the left and right line regions
            lineConst = (leftLineConst <= 0) | (rightLineConst <= 0);
            lineConst = (lineConst * -1) + 1;
            
            % TESTING: plot permitted regions
%             if leftSlope == 1
%                 plotLeft = @(x,y) -y + (slopeL*(x - pt(1))) + pt(2);
%             else
%                 plotLeft = @(x,y) y - (slopeL*(x - pt(1))) - pt(2);
%             end
%             if rightSlope == 1
%                 plotRight = @(x,y) -y + (slopeR*(x - pt(1))) + pt(2);
%             else
%                 plotRight = @(x,y) y - (slopeR*(x - pt(1))) - pt(2);
%             end
%             
%             [meshX,meshY] = meshgrid(1:.1:10,1:.1:10);
%             meshY = flipud(meshY);
%             pltOut = (plotLeft(meshX,meshY) <= 0) | (plotRight(meshX,meshY) <= 0);
%             pltOut = (pltOut * -1) + 1;
%             imshow(pltOut);
                     
%             plot(1:10,slopeL*([1:10] - pt(1)) + pt(2)); hold on;
%             plot(1:10,slopeR*([1:10] - pt(1)) + pt(2)); hold on;
%             scatter(positions(lineConst == 0,1),positions(lineConst == 0,2));
%             scatter(positions(lineConst == 1,1),positions(lineConst == 1,2));
%             axis([0 10 0 10]);
%             pause(0.01);
%             clf;
            
            c = [-maxVelocity + sqrt(vels(:,1).^2 + vels(:,2).^2);     % max vel component constraint
                 -maxAcceleration + sqrt(accs(:,1).^2 + accs(:,2).^2); % max acc component constraint
                 lineConst;
                 maxPosx - positions(end,1); 
                 maxPosy - positions(end,2);
                 -minPosx + positions(1,1);
                 -minPosy + positions(1,2)];   
        end
        
        %% Enforce smoothness
        % Constrain initial position to minX,minY and velocity to be zero
        ceq = [positions(1,1) - minPosx;
               positions(1,2) - minPosy;
               vels(1,1);
               vels(1,2)];
        
        for i = 1 : length(positions) - 1
            
            % The state at the beginning of the time int
            x_i = [positions(i,1);
                positions(i,2);
                vels(i,1);
                vels(i,2)];
            
            % What the state should be at the start of the next time int
            x_n = [positions(i+1,1);
                positions(i+1,2);
                vels(i+1,1);
                vels(i+1,2)];
            
            % The time derivative of the state at the beginning of the time int
            xdot_i = [vels(i,1);
                vels(i,2);
                accs(i,1);
                accs(i,2)];
            
            % The time derivative of the state at the end of the time int
            xdot_n = [vels(i+1,1);
                vels(i+1,2);
                accs(i+1,1);
                accs(i+1,2)];
            
            % The end state of the time interval calculated using quadrature
            xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
            
            % Constrain the end state of the current time interval to be
            % equal to the starting state of the next time interval
            ceq = [ceq ; abs(x_n - xend) - delta];
        end
        
        % Constrain end position to maxX,maxY and end velocity to 1
        ceq = [ceq ;
            positions(end,1) - maxPosx;
            positions(end,2) - maxPosy;
            vels(end,1);
            vels(end,2)];
    end

end