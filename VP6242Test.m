close all;
clc;
VP6242 = densoVP6242;
UR3robot = UR3e;
VP6242.model.base = transl([0.35,0,-0.281])*rpy2tr(0,0,0,'deg');
UR3robot.model.base = transl([-0.55,0,-0.15185])*rpy2tr(0,0,0,'deg');
VP6242.model.teach();% Open a menu to move the robot manually
% UR3robot.model.teach();

PassportStart = transl([-0.55,0.3,0.3])*rpy2tr(-90,0,-90,'deg');
PassportEnd = transl([-0.05,0,0])*rpy2tr(-90,90,-90,'deg');
qPassportStart = UR3robot.model.ikcon(PassportStart);
qPassportEnd = UR3robot.model.ikcon(PassportEnd);



qVPoriginal = [0 0 0 0 0 0 0 0]    ;                                % get current Joint Angles
%%
StampStart= transl([0.1,0,0])*rpy2tr(-180,0,-180,'deg');
StampEnd= transl([0,0,0])*rpy2tr(-180,0,-180,'deg');% get transform of current target brick
qstampStart = VP6242.model.ikcon(StampStart);                                  % get ik joint angles of current target brick
qstampStartLift= VP6242.model.ikcon(StampStart*transl([0,0,-0.05]));

qstampEnd = VP6242.model.ikcon(StampEnd);
qstampEndLift = VP6242.model.ikcon(StampEnd*transl([0,0,-0.05]));% get ik joint angles of current target brick

moveUR31 = moveTo(qPassportStart,qPassportEnd,UR3robot);
GetStamp = moveTo(qVPoriginal,qstampStart,VP6242);
Stamp1 = moveTo(qstampStart,qstampStartLift,VP6242);
Stamp2 = moveTo(qstampStartLift,qstampEndLift,VP6242);
Stamp3 = moveTo(qstampEndLift,qstampEnd,VP6242);
Stamp4 = moveTo(qstampEnd,qstampEndLift,VP6242);
moveUR32 = moveTo(qPassportEnd,qPassportStart,UR3robot);
Stamp5 = moveTo(qstampEndLift,qstampStartLift,VP6242);
Stamp6 = moveTo(qstampStartLift,qstampStart,VP6242);
Stamp7 = moveTo(qstampStart,qVPoriginal,VP6242);
%% 
function trajectory = moveTo(qStart,qEnd,Robot)
Start = Robot.model.fkineUTS(qStart);
End = Robot.model.fkineUTS(qEnd);
StartTr = Start(1:3,4);
EndTr = End(1:3,4);
travelDistance = norm(StartTr-EndTr)
trajectory = jtraj(qStart,qEnd,50);

for j = 1:size(trajectory,1)                                             % animate trajectory
    Robot.model.animate(trajectory(j,:));
    drawnow;
    pause(0.1);
end
end