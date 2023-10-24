close all;
clc;
robot = densoVP6242;
robot.model.teach();% Open a menu to move the robot manually
qcurrent = [0 0 19 0 0]                                    % get current Joint Angles
Tr= transl([0.2,0.2,0.155])*rpy2tr(0,0,0,'deg');                      % get transform of current target brick
qnew = robot.model.ikine(Tr);                                  % get ik joint angles of current target brick
qtraj = jtraj(qcurrent,qnew,50);                                    % make path trajectory
for j = 1:size(qtraj,1)                                             % animate trajectory
    robot.model.animate(qtraj(j,:));
    drawnow;
    pause(0.1);
end