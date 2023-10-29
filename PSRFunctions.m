classdef PSRFunctions


    properties
    end

    methods (Static)
        %% Environment
        % Brief: Function to load environment
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
        %% UR3Task
        % Brief: Move Robot Model and Object from start to end
        % Inputs: - qStart: Starting q angles
        %         - qEnd: Enfing q angles
        %         - Robot: Robot to be manipulated
        % Output: - Trajectory: array to of q angles get from qStart to qEnd
        %         - CurrentStep: Current Step performed by Trajectory
        function [trajectory,currentStep] = moveTo(qStart,qEnd,Robot)
            Start = Robot.model.fkineUTS(qStart);
            End = Robot.model.fkineUTS(qEnd);
            StartTr = Start(1:3,4);
            EndTr = End(1:3,4);
            travelDistance = norm(StartTr-EndTr)
            steps = round(100*travelDistance+15);

            trajectory = jtraj(qStart,qEnd,steps);

            for j = 1:size(trajectory,1)                                             % animate trajectory
                Robot.model.animate(trajectory(j,:));
                currentStep = j;
                drawnow;
                pause(0.1);
            end
        end
    end
end