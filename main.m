% ASSIGNMENT 2 MAIN 
% Project: Passport stamping using UR3 & VP6242
% Team: Kent Tran   - 11980099
%       Serey Te    - 14075814
%       Dan Pham    - 13945480
function [] = main()
%% Load Environment
PSRFunctions.Environment;
%% Load Robots
%Load UR3
UR3Robot = UR3(transl(0,0.3,0.7));
%Load VP6242
VP6242 = densoVP6242(transl(0,-0.3,0.6)*trotz(-pi/2));

%% Load Objects
%Load Passport
passport = PlaceObject('passport_ply.PLY',[0.15 0.22 1.00]);
passportverts = get(passport,'Vertices');
%Load Stamp
stamp = PlaceObject('passport_ply.PLY',[0.15 0.22 1.00]);
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
robot_q = robot.model.getpos;
initial_ee = robot.model.fkine(robot_q);

robot2_q = robot2.model.getpos;
% Passport moved to start location
%% UR3 Movement 1
% UR3 Pick Up Passport
% UR3 Move to stamp location
%% VP6242 Movement 2
% VP6242 pick up stamp,
end
