% ASSIGNMENT 2 MAIN 
% Project: Passport stamping using UR3 & VP6242
% Team: Kent Tran   - 11980099
%       Serey Te    - 14075814
%       Dan Pham    - 13945480
function [] = main()

clear all;
close all;
clc;
hold on;

%% Load Environment
PSRFunctions.Environment;
%% Load Robots
% Load UR3
UR3robot = UR3(transl(-0.2,0.20,0.70));
% Load VP6242
VP6robot = densoVP6242(transl(-0.2,-0.4,0.60)*trotz(-pi/2));

% Load Stamp
stamp = PlaceObject('stampper_ply.PLY',[-0.042 -0.12 0.71]);
StampVerts = get(stamp,'Vertices');

%% Define Locations and Positions
% **Define All Movement Positions**
QVP6Default = [0 0 0 0 0 0 0 0];
QUR3Default = [0 0 0 0 0 0];
VPInitialGuess = [-0.4876 0.7480 0.3158 0 0 0 0.4747 0];
URInitialGuess = [-0.4538 -0.0873 1.6580 1.571 1.1170 0];
UREndGuess = [1.7801 -0.6108 -2.3561 -0.1745 -1.309 0];

%Passport Positions
QPassportStart = UR3robot.model.ikcon(transl(0.08,0.22,1.00)*trotz(pi/2)*trotx(pi/2),URInitialGuess);
QPassportLift = UR3robot.model.ikcon(transl(-0.05,0.22,1.00)*trotz(pi/2)*trotx(pi/2),URInitialGuess);
QPassportEnd = UR3robot.model.ikcon(transl(-0.265,0,0.71)*trotz(pi/2)*trotx(pi/2)*troty(-pi/2),UREndGuess);

% Stamp Positions
QStampStart = VP6robot.model.ikcon(transl(-0.042,-0.12,0.71)*troty(pi),VPInitialGuess);
QStampStartLift = VP6robot.model.ikcon(transl(-0.042,-0.12,0.76) * troty(pi),VPInitialGuess);
QStampEndLift = VP6robot.model.ikcon(transl(-0.29,-0.05,0.76) * troty(pi),VPInitialGuess);
QStampEnd = VP6robot.model.ikcon(transl(-0.29,-0.05,0.72)* troty(pi),VPInitialGuess);

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
% Move Robots to operational position for system startup
        [StartVP0,CurrentStep] = PSRFunctions.moveTo(QVP6Default,QStampStartLift,VP6robot);
        [StartUR0,CurrentStep] = PSRFunctions.moveTo(QUR3Default,QPassportStart,UR3robot);

% Load First Passport
input('Press enter to Load New Passport')
[passport,PassportVerts] = PSRFunctions.loadPassport;
passportLoaded = 1;

while passportLoaded == 1

    [MoveUR1,CurrentStep,PassportVerts] = PSRFunctions.moveTo(QPassportStart,QPassportLift,UR3robot,passport,PassportVerts);
    [MoveUR2,CurrentStep,PassportVerts] = PSRFunctions.moveTo(QPassportLift,QPassportEnd,UR3robot,passport,PassportVerts);
    [MoveUR2,CurrentStep] = PSRFunctions.moveTo(QPassportEnd,QPassportLift,UR3robot);
    [moveVP1,CurrentStep] = PSRFunctions.moveTo(QStampStartLift,QStampStart,VP6robot);
    [moveVP2,CurrentStep,StampVerts] = PSRFunctions.moveTo(QStampStart,QStampStartLift,VP6robot,stamp,StampVerts);
    [moveVP3,CurrentStep,StampVerts] = PSRFunctions.moveTo(QStampStartLift,QStampEndLift,VP6robot,stamp,StampVerts);
    [moveVP4,CurrentStep,StampVerts] = PSRFunctions.moveTo(QStampEndLift,QStampEnd,VP6robot,stamp,StampVerts);
    [moveVP5,CurrentStep,StampVerts] = PSRFunctions.moveTo(QStampEnd,QStampEndLift,VP6robot,stamp,StampVerts);
    [moveVP6,CurrentStep,StampVerts] = PSRFunctions.moveTo(QStampEndLift,QStampStartLift,VP6robot,stamp,StampVerts);
    [moveVP7,CurrentStep,StampVerts] = PSRFunctions.moveTo(QStampStartLift,QStampStart,VP6robot,stamp,StampVerts);
    [moveVP8,CurrentStep] = PSRFunctions.moveTo(QStampStart,QStampStartLift,VP6robot);
    [MoveUR2,CurrentStep] = PSRFunctions.moveTo(QPassportLift,QPassportEnd,UR3robot);
    [MoveUR2,CurrentStep,PassportVerts] = PSRFunctions.moveTo(QPassportEnd,QPassportLift,UR3robot,passport,PassportVerts);
    [MoveUR2,CurrentStep,PassportVerts] = PSRFunctions.moveTo(QPassportLift,QPassportStart,UR3robot,passport,PassportVerts);

    % Animate Passport Removal
    for i= 1:25
        delete(passport);
        passport = PlaceObject('passport_ply.PLY',[(0.15+i*0.006) 0.22 1.00]);
        drawnow
        pause(0.1);
    end
    delete(passport);
    % Prompt to Load Passport and continue stamping operation
    EnterPassportPrompt = "Load Passport? Y/N: ";
    txt = input(EnterPassportPrompt,"s");
    if isempty(txt)
        txt = 'Y';
    end
    if txt == 'y' || txt == 'Y'
        [passport,PassportVerts] = PSRFunctions.loadPassport;
        passportLoaded = 1;
    else 
        passportLoaded = 0;
    end
end

% Move Robots to Default position for system shutdown.
[FinalVP0,CurrentStep] = PSRFunctions.moveTo(QStampStartLift,QVP6Default,VP6robot);
[FinalUR0,CurrentStep] = PSRFunctions.moveTo(QPassportStart,QUR3Default,UR3robot);
hold on;
end
