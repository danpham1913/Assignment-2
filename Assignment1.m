
clf
clc
clear

%Environment
Environment

%Robot LinearUR3
robot = densoVP6242(transl(-0.2,-0.4,0.60)*trotz(-pi/2));
robot_q = robot.model.getpos;
% robot.model.animate(robot_q);
initial_ee = robot.model.fkine(robot_q);
robot2 = UR3(transl(-0.15,0.20,0.65));
robot2_q = robot2.model.getpos;

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

% Resolved Motion Rate Control
steps = 50;

% x1 = [-0.15 0.22 1]';
% x2 = [0.15 0.22 1]';
% deltaT = 0.05;                                        % Discrete time step
% 
% x = zeros(3,steps);
% s = lspb(0,1,steps);                                 % Create interpolation scalar
% for i = 1:steps
%     x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
% end
% 
% 
% % 3.10
% for i = 1:steps-1
%     xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
%     J1_1 = robot2.model.jacob0(q1Matrix_1(i,:));            % Get the Jacobian at the current state
% %     J1_1 = J1_1(1:3,1:3);                          % Take only first 2 rows
%     qdot = inv(J1_1)*xdot;                             % Solve velocitities via RMRC
%     q1Matrix_1(i+1,:) =  q1Matrix_1(i,:) + deltaT*qdot';                   % Update next joint state
% end


%UR3
for j = 1:size(q1Matrix_1,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_1(j,:));
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
    updatepoint = robot2.model.fkine(q1Matrix_2(j,:)).T * trfix;
    trvert = updatepoint(1:3,:)';
    set(passport,'Vertices',trvert);
    drawnow(); 
end


for j = 1:size(q1Matrix_3,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_3(j,:));
    updatepoint = robot2.model.fkine(q1Matrix_3(j,:)).T * trfix;
    trvert = updatepoint(1:3,:)';
    set(passport,'Vertices',trvert);
    drawnow(); 
end

for j = 1:size(q1Matrix_4,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_4(j,:));
    drawnow(); 
end


%DensoVP6242
for j = 1:size(q2Matrix_1,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_1(j,:));
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    drawnow(); 
end

trfix2 = inv(robot.model.fkine(robot.model.getpos).T) * [vertsstamp, ones(size(vertsstamp,1),1)]';


for j = 1:size(q2Matrix_2,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_2(j,:));
%     tr = robot2.model.fkine(q1Matrix_1(j,:)).T;
%     transformVert = [vertspp1, ones(size(vertspp1,1),1)]*tr';
%     set(passport,'Vertices',transformVert(:,1:3));
    updatepoint2 = robot.model.fkine(q2Matrix_2(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end


for j = 1:size(q2Matrix_3,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_3(j,:));
    updatepoint2 = robot.model.fkine(q2Matrix_3(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_4,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_4(j,:));
    updatepoint2 = robot.model.fkine(q2Matrix_4(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_5,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_5(j,:));
    updatepoint2 = robot.model.fkine(q2Matrix_5(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_6,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_6(j,:));
    updatepoint2 = robot.model.fkine(q2Matrix_6(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_7,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_7(j,:));
    updatepoint2 = robot.model.fkine(q2Matrix_7(j,:)).T * trfix2;
    trvert2 = updatepoint2(1:3,:)';
    set(stamp,'Vertices',trvert2);
    drawnow(); 
end

for j = 1:size(q2Matrix_8,1)                                             % animate trajectory
    robot.model.animate(q2Matrix_8(j,:));
    drawnow(); 
end

%UR3 return path
for j = 1:size(q1Matrix_5,1)                                             % animate trajectory
    robot2.model.animate(q1Matrix_5(j,:));
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
