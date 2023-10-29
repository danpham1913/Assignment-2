
clf
clc
clear

%Environment
Environment

%Robot LinearUR3
robot = densoVP6242(transl(-0.25,-0.45,0.75));
robot_q = robot.model.getpos;
% robot.model.animate(robot_q);
initial_ee = robot.model.fkine(robot_q);
robot2 = UR3(transl(-0.2,0.25,0.8));
robot2_q = robot2.model.getpos;

%Placing Bricks
passport = PlaceObject('passport_ply.PLY',[0.20 0.25 1.21]);
vertspp1 = get(passport,'Vertices');
% set(brick1,'Vertices',vertsb1(:,1:3))

%UR3 movement path

% b1_ee0 = transl(0.20,0.25,1.21);
% b1_eeF = transl(-0.4,0.15,0.8);
qb1 = robot2.model.ikcon(transl(0.20,0.25,1.21) * troty(pi));
qb2 = robot2.model.ikcon(transl(-0.20,0.25,1.21)*troty(pi));


qb_end = robot2.model.ikcon(transl(-1.5,-0.5,0.8)* troty(pi));


steps = 50;

q_1 = jtraj(robot2_q,qb1,steps); %initial to brick
q_2 = jtraj(qb1,qb2,steps); %brick to end brick
q_3 = jtraj(qb2,robot2_q,steps); %end brick back to initial


for j = 1:size(q_1,1)                                             % animate trajectory
    robot2.model.animate(q_1(j,:));
    tr = robot2.model.fkine(q_1(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    drawnow(); 
end

trfix = inv(robot2.model.fkine(robot2.model.getpos).T) * [vertspp1, ones(size(vertspp1,1),1)]';


for j = 1:size(q_2,1)                                             % animate trajectory
    robot2.model.animate(q_2(j,:));
%     tr = robot.model.fkine(q_2(j,:)).T;
%     transformVertBrick1 = (tr * [vertsb1, ones(size(vertsb1,1),1)]')';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    updatepoint = robot2.model.fkine(q_2(j,:)).T * trfix;
    trvert = updatepoint(1:3,:)';
    set(passport,'Vertices',trvert);
    drawnow(); 
end

for j = 1:size(q_3,1)                                             % animate trajectory
    robot2.model.animate(q_3(j,:));
%     tr = robot.model.fkine(q_2(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    drawnow(); 
end
% robot.model.animate(q_2);
% robot.model.animate(q_3);
