classdef PSRFunctions


    properties
    end

    methods (Static)
        %% **Environment Function***
        % Brief: Function to load environment
        function Environment()
            hold on
            %Set up workspace
            axis ([-1 1.3 -1 1 0 2.5]);
            axis equal;
            xlabel ('X');
            ylabel ('Y');
            zlabel ('Z');
            grid on;
            PlaceObject('personFemaleBusiness.ply',[0.75 0 0]);
            PlaceObject('ColoredRoom.ply',[0 0 1]);
            % PlaceObject('fullroom_ply.ply',[0.175 0.1 1]);
            camlight
        end


        %% **moveTo Function***
        % Brief: Move Robot Model and Object from start to end
        % Inputs:   - qStart: Starting q angles
        %           - qEnd: Enfing q angles
        %           - Robot: Robot to be manipulated
        %           - Object: Optional Object to be moved with Robot
        %           - ObjectVerts: Object Data
        % Output:   - Trajectory: Array to of q angles get from qStart to qEnd
        %           - CurrentStep: Current Step performed by Trajectory
        %           - ObjectVerts: Object New Data returned.
        function [trajectory,currentStep,ObjectVerts] = moveTo(QStart,QEnd,Robot,Object,ObjectVerts)
            % Check if Object is provided
            if nargin > 4
                trfix = inv(Robot.model.fkine(Robot.model.getpos).T) * [ObjectVerts, ones(size(ObjectVerts,1),1)]';
                ObjectAttached = 1;
            else
                ObjectAttached = 0;
            end

            %Determine Steps required based on distance
            Start = Robot.model.fkineUTS(QStart);
            End = Robot.model.fkineUTS(QEnd);
            StartTr = Start(1:3,4);
            EndTr = End(1:3,4);
            travelDistance = norm(StartTr-EndTr);
            steps = round(100*travelDistance+15);

            %Generate Trajectory
            trajectory = jtraj(QStart,QEnd,steps);

            % animate trajectory
            for i = 1:size(trajectory,1)
                Robot.model.animate(trajectory(i,:));
                if ObjectAttached == 1
                    UpdateObject = Robot.model.fkine(trajectory(i,:)).T * trfix;
                    trvert = UpdateObject(1:3,:)';
                    set(Object,'Vertices',trvert);
                end

                currentStep = i;
                drawnow;
                pause(0);
            end

            %Return objectvertices so they are updated when called
            if ObjectAttached == 1
                ObjectVerts = get(Object,'Vertices');
            end
        end
       
        %% **loadPassport Function***
        % Brief: Load a passport and animate the slide in entry
        % Inputs: - Nil
        % Output: - passport: Return Passport Object
        %         - PassportVerts: Return Passport Object Data
        function [passport, PassportVerts] = loadPassport()
            passport = PlaceObject('passport_ply.PLY',[0.25 0.22 1.00]);

            steps = 25;
            for i= 1:steps
                delete(passport);
                passport = PlaceObject('passport_ply.PLY',[(0.25-i*0.004) 0.22 1.00]);
                drawnow;
                pause(0);
            end
            PassportVerts = get(passport,'Vertices');
        end
    end
end