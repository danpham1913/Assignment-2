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
            PlaceObject('fullroom2final.PLY',[0 0 1]);
            PlaceObject('personMaleOld.ply',[2.25 0 0]);
            PlaceObject('suitcase.ply',[2.25 0 0]);
            PlaceObject('suitcase.ply',[0.85 0.25 0]);
            PlaceObject('suitcase.ply',[1.5 1 0]);
            PlaceObject('fireExtinguisher.ply',[0 0.8 0]);
            PlaceObject('emergencyStopButton.ply',[-0.8 -0.8 1]);
            camlight
        end
        %% **Set Prisms Function***
        % Brief: Function to load environment
        function [Prismresults] = SetPrisms(UserGui)
            prismParams = {
                [0.1, 0.01, 0], [-0.47, -0.18, 0.7]; %Table
                [-0.02, -0.25, 0], [-0.32, -0.54, 0.60]; %VP Stand
                [-0.175, 0.265, 0], [-0.375, 0.465, 0.70]; %UR3 Stand
                [0.17, 0.650, 0], [0.12, -0.8, 1.98]; %Front Wall
                [0, 0.1, 0.8], [-0.4, -0.25, 0.9];
                };



            
            results = cell(1, size(prismParams,1));
            % Loop to create and store the results
            for i = 1:size(prismParams,1)
                [vertex, faces, faceNormals] = PSRFunctionsKent.RectangularPrism(prismParams{i, 1}, prismParams{i, 2});
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
                    UserGui.CollisionDetectedLampLabel.Text = 'Collision Detected';
                else
                    UserGui.CollisionDetectedLamp.Color = [0,1,0];
                    UserGui.CollisionDetectedLampLabel.Text = 'No Collision Detected';
                end
                if ObjectAttached == 1
                    if UserGui.Lamp_2.Color == [1,0,0];
                        PSRFunctionsKent.EStopFunction(UserGui);
                    elseif UserGui.DoorClosedLamp.Color == [1,0,0];
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

            while UserGui.DoorClosedLamp.Color == [1,0,0]
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
                tr = PSRFunctionsKent.GetLinkPoses(qMatrix(qIndex,:), robot);

                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && PSRFunctionsKent.whereIntersect(intersectP,vertex(faces(faceIndex,:)',:))
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

        %% **RectangularPrism Function***
        % Brief: Load a passport and animate the slide in entry
        % Inputs: - Nil
        % Output: - passport: Return Passport Object
        %         - PassportVerts: Return Passport Object Data
        function [vertex,face,faceNormals] = RectangularPrism(lower,upper,plotOptions,axis_h)
            if nargin<4
                axis_h=gca;
                if nargin<3
                    plotOptions.plotVerts=false;
                    plotOptions.plotEdges=true;
                    plotOptions.plotFaces=true;
                end
            end
            hold on

            vertex(1,:)=lower;
            vertex(2,:)=[upper(1),lower(2:3)];
            vertex(3,:)=[upper(1:2),lower(3)];
            vertex(4,:)=[upper(1),lower(2),upper(3)];
            vertex(5,:)=[lower(1),upper(2:3)];
            vertex(6,:)=[lower(1:2),upper(3)];
            vertex(7,:)=[lower(1),upper(2),lower(3)];
            vertex(8,:)=upper;

            face=[1,2,3;1,3,7;
                1,6,5;1,7,5;
                1,6,4;1,4,2;
                6,4,8;6,5,8;
                2,4,8;2,3,8;
                3,7,5;3,8,5;
                6,5,8;6,4,8];

            if 2 < nargout
                faceNormals = zeros(size(face,1),3);
                for faceIndex = 1:size(face,1)
                    v1 = vertex(face(faceIndex,1)',:);
                    v2 = vertex(face(faceIndex,2)',:);
                    v3 = vertex(face(faceIndex,3)',:);
                    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
            end
            %% If plot verticies
            if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
                for i=1:size(vertex,1);
                    plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
                    text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
                end
            end

            %% If you want to plot the edg
            if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
                links=[1,2;
                    2,3;
                    3,7;
                    7,1;
                    1,6;
                    5,6;
                    5,7;
                    4,8;
                    5,8;
                    6,4;
                    4,2;
                    8,3];

                for i=1:size(links,1)
                    plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
                        [vertex(links(i,1),2),vertex(links(i,2),2)],...
                        [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
                end
            end

            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                tcolor = [.2 .2 .8];

                patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');
            end

        end
        function [transforms] = GetLinkPoses(q,robot)

            links = robot.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.model.base;

            for i = 1:length(links)
                L = links(1,i);

                current_transform = transforms(:,:, i);

                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end
    end
end








