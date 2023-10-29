function GripperTest()
clc;
close all;

gripperArm;
r = UR3;
hold on
% moveArm(0,100,gripperArm);
ToP = 20;
FromP = 100;

% function moveArm(FromP, ToP, gripper)
steps = 50;
qRNew = [deg2rad((-3/10)*ToP + 90), deg2rad((3/10)*ToP)]
qLNew = -qRNew 
qROld = [deg2rad((-3/10)*FromP + 90), deg2rad((3/10)*FromP)]
qLOld = -qROld

LTrajectory = jtraj(qLOld, qLNew, steps);
RTrajectory = jtraj(qROld, qRNew, steps);

left.base = r.model.fkine(r.model.getpos()).T * trotx(deg2rad(-90));
right.base = r.model.fkine(r.model.getpos()).T * trotx(deg2rad(-90));
left.plot(qLOld, 'noname', 'nowrist');
right.plot(qROld, 'noname', 'nowrist');
%%
for i = 1:steps
    left.animate(LTrajectory(i,:));
    right.animate(RTrajectory(i,:));
    drawnow();
    pause(0);
end
end



