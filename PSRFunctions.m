classdef Test


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
        function [status] = UR3Task(stamped)

        end

        %% VP6242Task
        % Brief: Function to execute UR3 movements and report back when complete.
        % Inputs: Stamped   - status if passport has been stamped.
        % Output: Status    - 0:Movement incomplete
        %                   - 1: Stamp picked up and passport stamped.
        function [status] = VP6242Task()
        
        
            function [QMatrix] = Tragectory(startQ, EndQ, Steps)

        end
    end
end