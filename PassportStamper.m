% ASSIGNMENT 2 MAIN 
% Project: Passport stamping using UR3 & VP6242
% Team: Kent Tran   - 11980099
%       Serey Te    -
function [] = PassportStamper()
%% Load Environment
figure;
hold on;
title('Environment');
xlabel('X');
ylabel('y');
zlabel('z');
axis([-2,2,-2,2,0,2]);                                                      % Set workspace size
%% Load Robots
%Load UR3
%Load VP6242
VP6242 = densoVP6242;                                                          % initialize robot as LiUR3
densoVP6242.model.base = transl([0,0,0])*rpy2tr(0,0,0,'deg');             % set base position


%% Visualise components


%% Load Objects
%Load Passport
%Load Stamp

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
% Passport moved to start location
%% UR3 Movement 1
% UR3 Pick Up Passport
% UR3 Move to stamp location
%% VP6242 Movement 2
% VP6242 pick up stamp,
end
