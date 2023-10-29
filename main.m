% ASSIGNMENT 2 MAIN 
% Project: Passport stamping using UR3 & VP6242
% Team: Kent Tran   - 11980099
%       Serey Te    - 14075814
%       Dan Pham    - 13945480
function [] = main()
%% Load Environment
PSRFunctions.Environment;
% % Load Robots
%Load UR3
UR3Robot = UR3(transl(0,0.3,0.7));
%Load VP6242
VP6242 = densoVP6242(transl(0,-0.3,0.6)*trotz(-pi/2));
VP6242.model.teach();% Open a menu to move the robot manually
% % Load Objects
%Load Passport
passport = PlaceObject('passport_ply.PLY',[0.325 0.3125 1]);
passportverts = get(passport,'Vertices');
%Load Stamp
stamp = PlaceObject('stampper_ply.PLY',[0.1315 -0.02 0.707]);
stampVerts = get(passport,'Vertices');

%% Establish Flags
%Passport Location Flag

%% Establish Safety Checks (While loops)
% Collision Check
    %SafetyFunctions.CollisionCheck
% Emergency Button Check
    %SafetyFunctions.EmergencyButton
% Light Guard
    %SafetyFunctions.LightGuard
% Door Reed Switch
    %SafetyFunctions.DoorOpen

%% Perform Tasks
robot_q = VP6242.model.getpos;
initial_ee = VP6242.model.fkine(robot_q);

robot2_q = UR3Robot.model.getpos;
% Passport moved to start location
%% UR3 Movement 1
trfix = inv(UR3Robot.model.fkine(UR3Robot.model.getpos).T) * [passportverts, ones(size(passportverts,1),1)]';
% UR3 Pick Up Passport

% UR3 Move to stamp location
qVPoriginal = [0 0 0 0 0 0 0 0]; %VP6242 Original Position
StampStart= transl([0.1315 -0.02 0.71])*rpy2tr(-180,0,-180,'deg');
StampEnd= transl([-0.14,0,0.72])*rpy2tr(-180,0,-180,'deg');% get transform of current target brick

qstampStart = VP6242.model.ikcon(StampStart);
qstampStartLift= VP6242.model.ikcon(StampStart*transl([0,0,-0.05]));
qstampEnd = VP6242.model.ikcon(StampEnd);
qstampEndLift = VP6242.model.ikcon(StampEnd*transl([0,0,-0.05]));

[moveVP1,CurrentStep] = PSRFunctions.moveTo(qstampStart,qstampStartLift,VP6242);
[moveVP2,CurrentStep] = PSRFunctions.moveTo(qstampStartLift,qstampEndLift,VP6242);
[moveVP3,CurrentStep] = PSRFunctions.moveTo(qstampEndLift,qstampEnd,VP6242);
[moveVP4,CurrentStep] = PSRFunctions.moveTo(qstampEnd,qstampEndLift,VP6242);
[moveVP5,CurrentStep] = PSRFunctions.moveTo(qstampEndLift,qstampStartLift,VP6242);
[moveVP6,CurrentStep] = PSRFunctions.moveTo(qstampStartLift,qstampStart,VP6242);
%% VP6242 Movement 2
% VP6242 pick up stamp,
end
