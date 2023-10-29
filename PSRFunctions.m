classdef PSRFunctions


    properties
    end

    methods (Static)
        %% UR3Task
        % Brief: Function to execute UR3 movements (pick and place and report back when complete.
        % Inputs: Stamped   - status if passport has been stamped.
        % Output: Status    - 0:Movement incomplete
        %                   - 1:Passport picked up and moved to stamping
        %                       location completed.
        %                   - 2:passport moved back to original location
        %                       and drop off complete.
        function Environment()
            hold on
            %Set up workspace
            axis equal;
            xlabel ('X');
            ylabel ('Y');
            zlabel ('Z');
            grid on;
            PlaceObject('fullroom_ply.ply',[0.175 0.1 1]);
            camlight
        end
    end
end