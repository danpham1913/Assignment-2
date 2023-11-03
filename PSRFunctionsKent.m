classdef PSRFunctionsKent


    properties
    end

    methods (Static)
        %% **Environment Function***
        % Brief: Function to load environment
        function Environment()
            hold on
            %Set up workspace
            axis ([-2 4 -1.5 1.5 0 3])
            axis equal;
            xlabel ('X');
            ylabel ('Y');
            zlabel ('Z');
            grid on;

            %Set up walls
            surf([-2,-2;4,4],[-1.5,1.5;-1.5,1.5],[0.01,0.01;0.01,0.01],'CData',imread('floor.jpg'),'FaceColor','texturemap');
            surf([-2,4;-2,4],[-1.5,-1.5;-1.5,-1.5],[3,3;0.01,0.01],'CData',imread('Screenshot(2).jpg'),'FaceColor','texturemap');
            surf([4,4;4,4],[-1.5,1.5;-1.5,1.5],[3,3;0.01,0.01],'CData',imread('Screenshot(3).jpg'),'FaceColor','texturemap');
            surf([-0.6,-0.4;-0.6,-0.4],[-0.85,-0.85;-0.85,-0.85],[1,1;1.2,1.2],'CData',imread('Warning.jpg'),'FaceColor','texturemap');

            %Add objects
            PlaceObject('personMaleCasual.ply',[0.85 0 0]);
            PlaceObject('personFemaleBusiness.ply',[1.5 0 0]);
            PlaceObject('fullroom2_ply (1).PLY',[0 0 1]);
            PlaceObject('personMaleOld.ply',[2.25 0 0]);
            PlaceObject('suitcase.ply',[2.25 0 0]);
            PlaceObject('suitcase2.ply',[0.85 0.25 0]);
            PlaceObject('suitcase.ply',[1.5 1 0]);
            PlaceObject('fireExtinguisher.ply',[0 0.8 0]);
            PlaceObject('emergencyStopButton.ply',[-0.8 -0.8 1]);
            camlight
        end
                %% **Set Prisms Function***
        % Brief: Function to load environment
        function [Prismresults] = SetPrisms()
            prismParams = {
                [0.1, 0.025, 0], [-0.5, -0.2, 0.69]; %Table
                [-0.02, -0.23, 0], [-0.35, -0.54, 0.60]; %VP Stand
                [-0.15, 0.18, 0], [-0.35, 0.39, 0.71]; %UR3 Stand
                [0.17, 0.650, 0], [-0.1, -0.8, 1.05]; %Bottom Front Wall
                [0.17, 0.650, 1.05], [-0.1, -0.8, 1.99]; %Top Front Wall
                };
            results = cell(1, 5);
            % Loop to create and store the results
            for i = 1:5
                [vertex, faces, faceNormals] = RectangularPrism(prismParams{i, 1}, prismParams{i, 2});
                results{i} = struct('vertex', vertex, 'faces', faces, 'faceNormals', faceNormals);
            end
            Prismresults = results;
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
        function [ObjectVerts] = moveTo(QStart,QEnd,Robot,Robot2,UserGui,Prisms,Object,ObjectVerts)
            % Check if Object is provided
            if nargin > 6
                trfix = inv(Robot.model.fkine(Robot.model.getpos).T) * [ObjectVerts, ones(size(ObjectVerts,1),1)]';
                ObjectAttached = 1;
                UserGui.CollisionDetectedLamp.Color = [1,0,0];
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
            Collisionresult = 0;
            %Generate Trajectory
            trajectory = jtraj(QStart,QEnd,steps);

            % animate trajectory
            for i = 1:size(trajectory,1)
                Robot.model.animate(trajectory(i,:));
                for j  = 1:size(Prisms,1)
                    Prism = Prisms{j};
                    vertexi = Prism.vertex;
                    facesi = Prism.faces;
                    faceNormals = Prism.faceNormals;

                    Collisionresult = PSRFunctionsKent.IsCollision(Robot,trajectory(i,:),facesi,vertexi,faceNormals,false);
                    if Collisionresult == 1
                        break
                    end
                end
                if Collisionresult ==1
                    UserGui.CollisionDetectedLamp.Color = [1,0,0];
                else
                    UserGui.CollisionDetectedLamp.Color = [0,1,0];
                end
                if ObjectAttached == 1
                    if UserGui.Lamp_2.Color == [1,0,0];
                        PSRFunctionsKent.EStopFunction(UserGui);
                    elseif UserGui.DoorOpenLamp.Color == [1,0,0];
                        PSRFunctionsKent.DoorOpenFunction(Robot,Robot2,UserGui,Object,ObjectVerts)
                    end
                    UpdateObject = Robot.model.fkine(trajectory(i,:)).T * trfix;
                    trvert = UpdateObject(1:3,:)';
                    set(Object,'Vertices',trvert);
                    ObjectVerts = get(Object,'Vertices');   %Return objectvertices so they are updated when called
                else
                    if UserGui.Lamp_2.Color == [1,0,0];
                        PSRFunctionsKent.EStopFunction(UserGui);
                    end
                end
                drawnow;
                pause(0);
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

        %% **DoorOpen Function***
        % Brief: Load a passport and animate the slide in entry
        % Inputs: - Nil
        % Output: - passport: Return Passport Object
        %         - PassportVerts: Return Passport Object Data
        function DoorOpenFunction(Robot,Robot2,UserGui,Object,ObjectVerts)
            if nargin > 3
                trfix = inv(Robot.model.fkine(Robot.model.getpos).T) * [ObjectVerts, ones(size(ObjectVerts,1),1)]';
                ObjectAttached = 1;
            else
                ObjectAttached = 0;
            end
            QStart1 = Robot.model.getpos;
            QEnd1 = zeros(size(QStart1,1),1);
            QStart2 = Robot2.model.getpos;
            QEnd2 = zeros(size(QStart2,1),1);
            trajectory1 = jtraj(QStart1,QEnd1,100);
            trajectory2 = jtraj(QStart2,QEnd2,100);
            for i = 1:size(trajectory1,1)
                Robot.model.animate(trajectory1(i,:));
                if ObjectAttached == 1
                    UpdateObject = Robot.model.fkine(trajectory1(i,:)).T * trfix;
                    trvert = UpdateObject(1:3,:)';
                    set(Object,'Vertices',trvert);
                end
                drawnow;
                pause(0);
            end
            for i = 1:size(trajectory2,1)
                Robot2.model.animate(trajectory2(i,:));
                drawnow;
                pause(0);
            end

            while UserGui.DoorOpenLamp.Color == [1,0,0]
                pause(5);
            end
            for i = size(trajectory1,1):-1:1
                Robot.model.animate(trajectory1(i,:));
                if ObjectAttached == 1
                    UpdateObject = Robot.model.fkine(trajectory1(i,:)).T * trfix;
                    trvert = UpdateObject(1:3,:)';
                    set(Object,'Vertices',trvert);
                end
                drawnow;
                pause(0);
            end
            for i = size(trajectory2,1):-1:1
                Robot2.model.animate(trajectory2(i,:));
                drawnow;
                pause(0);
            end
        end
        %% **EStop Function***
        % Brief: Load a passport and animate the slide in entry
        % Inputs: - Nil
        % Output: - passport: Return Passport Object
        %         - PassportVerts: Return Passport Object Data
        function EStopFunction(UserGui)
            while UserGui.Lamp_2.Color == [1,0,0]
                pause(5);
            end
        end

        %% **WhereIntersect Function***
        % Brief: Load a passport and animate the slide in entry
        % Inputs: - Nil
        % Output: - passport: Return Passport Object
        %         - PassportVerts: Return Passport Object Data
        function result = whereIntersect(intersectP,triangleVerts)
            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);
            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);
            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);
            D = uv * uv - uu * vv;
            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end
            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end
            result = 1;                      % intersectP is in Triangle
        end

        %% **IsCollision Function***
        % Brief: Load a passport and animate the slide in entry
        % Inputs: - Nil
        % Output: - passport: Return Passport Object
        %         - PassportVerts: Return Passport Object Data
        function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = GetLinkPoses(qMatrix(qIndex,:), robot);

                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && PSRFunctionsKent.whereIntersect(intersectP,vertex(faces(faceIndex,:)',:))
                            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                            result = true;
                            disp('collision');
                            if returnOnceFound
                                return
                            end
                        end
                    end
                end
            end
        end
    end
end








