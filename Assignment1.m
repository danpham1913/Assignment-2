
clf
clc
clear

%Environment
Environment

%Robot LinearUR3
robot = densoVP6242(transl(1.5,0,0.8));
robot_q = robot.model.getpos;
% robot.model.animate(robot_q);
initial_ee = robot.model.fkine(robot_q);
% robot2 = densoVP6242(transl(1.5,0,0.8));


%Placing Bricks
brick1 = PlaceObject('HalfSizedRedGreenBrick.ply',[1.25 0 0.85]);
vertsb1 = get(brick1,'Vertices');
% set(brick1,'Vertices',vertsb1(:,1:3))

% b1_ee0 = transl(0.52,0,0.85);
% b1_eeF = transl(2,0,1);
qGuest = [0 0 0 0 0 0 0];
qb1 = robot.model.ikcon(transl(1.25,0,0.85) * troty(pi));
qb2 = robot.model.ikcon(transl(1.5,-0.5,0.85)* troty(pi));

steps = 50;

q_1 = jtraj(robot_q,qb1,steps); %initial to brick
q_2 = jtraj(qb1,qb2,steps); %brick to end brick
q_3 = jtraj(qb2,robot_q,steps); %end brick back to initial


for j = 1:size(q_1,1)                                             % animate trajectory
    robot.model.animate(q_1(j,:));
    tr = robot.model.fkine(q_1(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    drawnow(); 
end

trfix = inv(robot.model.fkine(robot.model.getpos).T) * [vertsb1, ones(size(vertsb1,1),1)]';


for j = 1:size(q_2,1)                                             % animate trajectory
    robot.model.animate(q_2(j,:));
%     tr = robot.model.fkine(q_2(j,:)).T;
%     transformVertBrick1 = (tr * [vertsb1, ones(size(vertsb1,1),1)]')';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    transbrick = robot.model.fkine(q_2(j,:)).T * trfix;
    trvert = transbrick(1:3,:)';
    set(brick1,'Vertices',trvert);
    drawnow(); 
end

for j = 1:size(q_3,1)                                             % animate trajectory
    robot.model.animate(q_3(j,:));
%     tr = robot.model.fkine(q_2(j,:)).T;
%     transformVertBrick1 = [vertsb1, ones(size(vertsb1,1),1)]*tr';
%     set(brick1,'Vertices',transformVertBrick1(:,1:3));
    drawnow(); 
end
% robot.model.animate(q_2);
% robot.model.animate(q_3);
