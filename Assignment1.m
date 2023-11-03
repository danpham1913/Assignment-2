
clf
clc
clear

%Environment
Environment

%Robot Denso VP6242
robot = densoVP6242(transl(-0.2,-0.4,0.60)*trotz(-pi/2));
robot_q = robot.model.getpos;
% robot.model.animate(robot_q);
initial_ee = robot.model.fkine(robot_q);


%plot prism

plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism([0.1,0.025,0], [-0.5,-0.2,0.69]); %stamping shelf
[vertex2,faces2,faceNormals2] = RectangularPrism([-0.02,-0.23,0], [-0.35,-0.54,0.60]); %denso VP6242 pillar
[vertex3,faces3,faceNormals3] = RectangularPrism([-0.15,0.18,0], [-0.35,0.39,0.71]); %UR3 pillar
[vertex4,faces4,faceNormals4] = RectangularPrism([0.17,0.650,0], [0.13,-0.8,0.95]); %bottom wall
[vertex5,faces5,faceNormals5] = RectangularPrism([0.17,0.650,1.05], [0.13,-0.8,1.99]);%top wall
axis equal
camlight


%Plot robot collision detection
robot_tr = zeros(4,4,robot.model.n+1);
robot_tr(:,:,1) = robot.model.base;
L = robot.model.links;
for i = 1 : robot.model.n
    robot_tr(:,:,i+1) = robot_tr(:,:,i) * trotz(robot_q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end


%collision detection algorithm
for i = 1 : size(robot_tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,robot_tr(1:3,4,i)',robot_tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            disp('Intersection');
        end
    end    
end






%Robot UR3
robot2 = UR3(transl(-0.15,0.20,0.65));
robot2_q = robot2.model.getpos;


%Plot robot collision detection
robot2_tr = zeros(4,4,robot.model.n+1);
robot2_tr(:,:,1) = robot.model.base;
L = robot.model.links;
for i = 1 : robot.model.n
    robot2_tr(:,:,i+1) = robot2_tr(:,:,i) * trotz(robot_q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end


%collision detection algorithm
for i = 1 : size(robot2_tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,robot2_tr(1:3,4,i)',robot2_tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            disp('Intersection');
        end
    end    
end


%Placing Passport
passport = PlaceObject('passport_ply.PLY',[0.15 0.22 1.00]);
vertspp1 = get(passport,'Vertices');

%Placing Stamp
stamp = PlaceObject('stampper_ply.PLY',[-0.042 -0.12 0.71]);
vertsstamp = get(stamp,'Vertices');


%UR3 movement path

%joint position
% transl(0.20,0.25,1.21) to transl(-0.4,0.15,0.8);
q1_1 = robot2.model.ikcon(transl(0.15,0.22,1.00) * troty(pi),[1.2467   -0.7979   -1.9448   -3.3411    6.1336   -0.2992]);
q1_2 = robot2.model.ikcon(transl(-0.15,0.22,1.00)*troty(pi),[1.2467 -0.7979 -1.9448 -3.3411 6.1336 -0.2992]);
q1_end = robot2.model.ikcon(transl(-0.26,-0.05,0.71)* troty(pi));

steps = 50;

%path 1
q1Matrix_1 = jtraj(robot2_q,q1_1,steps);
q1Matrix_2 = jtraj(q1_1,q1_2,steps);
q1Matrix_3 = jtraj(q1_2,q1_end,steps); 
q1Matrix_4 = jtraj(q1_end,robot2_q,steps);
q1Matrix_5 = jtraj(robot2_q,q1_end,steps);
q1Matrix_6 = jtraj(q1_end,q1_2,steps);
q1Matrix_7 = jtraj(q1_2,q1_1,steps);
q1Matrix_8 = jtraj(q1_1,robot2_q,steps);
%path 2

%Denso VP6242 movement path

%point position
q2_1 = robot.model.ikcon(transl(-0.042,-0.12,0.71) * troty(pi),[-0.4876 0.7480 0.3158 0 0 0 0.4747 0]);
q2_2 = robot.model.ikcon(transl(-0.042,-0.12,0.81) * troty(pi),[-0.4876 0.7480 0.3158 0 0 0 0.4747 0]);
q2_3 = robot.model.ikcon(transl(-0.26,-0.05,0.81) * troty(pi),[-0.4876 0.7480 0.3158 0 0 0 0.4747 0]);
q2_end = robot.model.ikcon(transl(-0.26,-0.05,0.71)* troty(pi),[-0.4876 0.7480 0.3158 0 0 0 0.4747 0]);

q2Matrix_1 = jtraj(robot_q,q2_1,steps);
q2Matrix_2 = jtraj(q2_1,q2_2,steps); 
q2Matrix_3 = jtraj(q2_2,q2_3,steps);
q2Matrix_4 = jtraj(q2_3,q2_end,steps); 
q2Matrix_5 = jtraj(q2_end,q2_3,steps);
q2Matrix_6 = jtraj(q2_3,q2_2,steps);
q2Matrix_7 = jtraj(q2_2,q2_1,steps);
q2Matrix_8 = jtraj(q2_1,robot_q,steps);




% 2.6: Go through until there are no step sizes larger than 1 degree
% q1 = [-pi/4,0,0];
% q2 = [pi/4,0,0];
% steps = 2;
% while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
%     steps = steps + 1;
% end
% qMatrix = jtraj(q1,q2,steps);

% 2.7
% result = true(steps,1);
% for i = 1: steps
%     result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
%     robot.animate(qMatrix(i,:));
% end
% end
% result = collide or not collide 0=collide
result = true(steps,1);

%UR3
for j = 1:size(q1Matrix_1,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_1(j,:));
    result(j) = IsCollision(robot2,q1Matrix_1(j,:),faces,vertex,faceNormals,false);
    result(j) = IsCollision(robot2,q1Matrix_1(j,:),faces2,vertex2,faceNormals2,false);
    result(j) = IsCollision(robot2,q1Matrix_1(j,:),faces3,vertex3,faceNormals3,false);
    result(j) = IsCollision(robot2,q1Matrix_1(j,:),faces4,vertex4,faceNormals4,false);
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    drawnow(); 
end

trfix = inv(robot2.model.fkine(robot2.model.getpos).T) * [vertspp1, ones(size(vertspp1,1),1)]';


for j = 1:size(q1Matrix_2,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_2(j,:));
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVert = [vertspp1, ones(size(vertspp1,1),1)]*tr';
%     set(passport,'Vertices',transformVert(:,1:3));

    %collision check
    result(j) = IsCollision(robot2,q1Matrix_2(j,:),faces,vertex,faceNormals,false);
    result(j) = IsCollision(robot2,q1Matrix_2(j,:),faces2,vertex2,faceNormals2,false);
    result(j) = IsCollision(robot2,q1Matrix_2(j,:),faces3,vertex3,faceNormals3,false);
    result(j) = IsCollision(robot2,q1Matrix_2(j,:),faces4,vertex4,faceNormals4,false);
    updatepoint = robot2.model.fkine(q1Matrix_2(j,:)).T * trfix;
    trvert = updatepoint(1:3,:)';
    set(passport,'Vertices',trvert);
    drawnow(); 
end


for j = 1:size(q1Matrix_3,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_3(j,:));
    %collision check
    result(j) = IsCollision(robot2,q1Matrix_3(j,:),faces,vertex,faceNormals,false);
    result(j) = IsCollision(robot2,q1Matrix_3(j,:),faces2,vertex2,faceNormals2,false);
    result(j) = IsCollision(robot2,q1Matrix_3(j,:),faces3,vertex3,faceNormals3,false);
    result(j) = IsCollision(robot2,q1Matrix_3(j,:),faces4,vertex4,faceNormals4,false);

    updatepoint = robot2.model.fkine(q1Matrix_3(j,:)).T * trfix;
    trvert = updatepoint(1:3,:)';
    set(passport,'Vertices',trvert);
    drawnow(); 
end

for j = 1:size(q1Matrix_4,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_4(j,:));
    %collision check
    result(j) = IsCollision(robot2,q1Matrix_4(j,:),faces,vertex,faceNormals,false);
    result(j) = IsCollision(robot2,q1Matrix_4(j,:),faces2,vertex2,faceNormals2,false);
    result(j) = IsCollision(robot2,q1Matrix_4(j,:),faces3,vertex3,faceNormals3,false);
    result(j) = IsCollision(robot2,q1Matrix_4(j,:),faces4,vertex4,faceNormals4,false);
    drawnow(); 
end


%DensoVP6242

% while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
%     steps = steps + 1;
% end

for j = 1:size(q2Matrix_1,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_1(j,:));
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
        result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    
    drawnow(); 
end

trfix2 = inv(robot.model.fkine(robot.model.getpos).T) * [vertsstamp, ones(size(vertsstamp,1),1)]';


for j = 1:size(q2Matrix_2,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_2(j,:));
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVert = [vertspp1, ones(size(vertspp1,1),1)]*tr';
%     set(passport,'Vertices',transformVert(:,1:3));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    updatepoint2 = robot.model.fkine(q2Matrix_2(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end


for j = 1:size(q2Matrix_3,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_3(j,:));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false); %collision check
    updatepoint2 = robot.model.fkine(q2Matrix_3(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_4,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_4(j,:));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    updatepoint2 = robot.model.fkine(q2Matrix_4(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_5,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_5(j,:));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    updatepoint2 = robot.model.fkine(q2Matrix_5(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_6,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_6(j,:));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    updatepoint2 = robot.model.fkine(q2Matrix_6(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_7,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_7(j,:));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    updatepoint2 = robot.model.fkine(q2Matrix_7(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_8,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_8(j,:));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    drawnow(); 
end

for j = 1:size(q1Matrix_5,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_5(j,:));
    result(j) = IsCollision(robot,q2Matrix_1(j,:),faces,vertex,faceNormals,false);
    drawnow(); 
end

% trfix = inv(robot2.model.fkine(robot2.model.getpos).T) * [vertspp1, ones(size(vertspp1,1),1)]';

for j = 1:size(q1Matrix_6,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_6(j,:));
    updatepoint = robot2.model.fkine(q1Matrix_6(j,:)).T * trfix;
    trvert = updatepoint(1:3,:)';
    set(passport,'Vertices',trvert);
    drawnow(); 
end

for j = 1:size(q1Matrix_7,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_7(j,:));
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVert = [vertspp1, ones(size(vertspp1,1),1)]*tr';
%     set(passport,'Vertices',transformVert(:,1:3));
    updatepoint = robot2.model.fkine(q1Matrix_7(j,:)).T * trfix;
    trvert = updatepoint(1:3,:)';
    set(passport,'Vertices',trvert);
    drawnow(); 
end

for j = 1:size(q1Matrix_8,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_8(j,:));
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    drawnow(); 
end