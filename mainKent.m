% ASSIGNMENT 2 MAIN
% Project: Passport stamping using UR3 & VP6242
% Team: Kent Tran   - 11980099
%       Serey Te    - 14075814
%       Dan Pham    - 13945480
function [] = mainKent()

clear all;
close all;
clc;
hold on;

%% Load Environment

UserGui = GuiApp;

PSRFunctionsKent.Environment;

%SetUp Collison Detections in Environment
[Prisms] = PSRFunctionsKent.SetPrisms(UserGui);
% result = collide or not collide 0=collide
%% Load Robots
% Load UR3
UR3robot = UR3new(transl(-0.25,0.35,0.70));
UR3robot.model.teach();
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
URInitialGuess = [-2.8797 0.7853 -2.1816 -1.7453 -1.3090 0];
UREndGuess = [-4.1016 -0.5236 -2.5307 -0.1745 -0.9599 0];

%Passport Positions
QPassportStart = UR3robot.model.ikcon(transl(-0.025,0.22,1.00)*trotz(pi/2)*trotx(pi/2),URInitialGuess);
QPassportLift = UR3robot.model.ikcon(transl(-0.18,0.22,1.00)*trotz(pi/2)*trotx(pi/2),URInitialGuess);
QPassportEnd = UR3robot.model.ikcon(transl(-0.265,0.10,0.71)*trotz(pi/2)*trotx(pi/2)*troty(-pi/2),UREndGuess);

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
PSRFunctionsKent.moveTo(QVP6Default,QStampStartLift,VP6robot,UR3robot,UserGui,Prisms);
PSRFunctionsKent.moveTo(QUR3Default,QPassportStart,UR3robot,VP6robot,UserGui,Prisms);

Startup = 1;
while strcmp(UserGui.SystemSwitch.Value, 'On') || Startup == 1
    % Prompt to Load Passport and continue stamping operation
    if strcmp(UserGui.SystemSwitch.Value, 'On')
        Startup = 0;
    end
    if UserGui.DoorClosedLamp.Color == [1,0,0];
        PSRFunctionsKent.DoorOpenFunction(VP6robot,UR3robot,UserGui);
    end
    if strcmp(UserGui.SystemSwitch.Value, 'On') && UserGui.LoadNewPassportsButton.Value == 0
        %Load New Passport
        [passport,PassportVerts] = PSRFunctionsKent.loadPassport;
        UserGui.SystemStatusEditField.Value = 'Passport Entered';
        %Perform Stamping Motions
        [PassportVerts] = PSRFunctionsKent.moveTo(QPassportStart,QPassportLift,UR3robot,VP6robot,UserGui,Prisms,passport,PassportVerts);
        [PassportVerts] = PSRFunctionsKent.moveTo(QPassportLift,QPassportEnd,UR3robot,VP6robot,UserGui,Prisms,passport,PassportVerts);
        PSRFunctionsKent.moveTo(QPassportEnd,QPassportLift,UR3robot,VP6robot,UserGui,Prisms);
        UserGui.SystemStatusEditField.Value = 'Stamping Passport';
        PSRFunctionsKent.moveTo(QStampStartLift,QStampStart,VP6robot,UR3robot,UserGui,Prisms);
        [StampVerts] = PSRFunctionsKent.moveTo(QStampStart,QStampStartLift,VP6robot,UR3robot,UserGui,Prisms,stamp,StampVerts);
        [StampVerts] = PSRFunctionsKent.moveTo(QStampStartLift,QStampEndLift,VP6robot,UR3robot,UserGui,Prisms,stamp,StampVerts);
        [StampVerts] = PSRFunctionsKent.moveTo(QStampEndLift,QStampEnd,VP6robot,UR3robot,UserGui,Prisms,stamp,StampVerts);
        UserGui.SystemStatusEditField.Value = 'Passport Stamped';
        [StampVerts] = PSRFunctionsKent.moveTo(QStampEnd,QStampEndLift,VP6robot,UR3robot,UserGui,Prisms,stamp,StampVerts);
        [StampVerts] = PSRFunctionsKent.moveTo(QStampEndLift,QStampStartLift,VP6robot,UR3robot,UserGui,Prisms,stamp,StampVerts);
        [StampVerts] = PSRFunctionsKent.moveTo(QStampStartLift,QStampStart,VP6robot,UR3robot,UserGui,Prisms,stamp,StampVerts);
        PSRFunctionsKent.moveTo(QStampStart,QStampStartLift,VP6robot,UR3robot,UserGui,Prisms);
        PSRFunctionsKent.moveTo(QPassportLift,QPassportEnd,UR3robot,VP6robot,UserGui,Prisms);
        [PassportVerts] = PSRFunctionsKent.moveTo(QPassportEnd,QPassportLift,UR3robot,VP6robot,UserGui,Prisms,passport,PassportVerts);
        [PassportVerts] = PSRFunctionsKent.moveTo(QPassportLift,QPassportStart,UR3robot,VP6robot,UserGui,Prisms,passport,PassportVerts);
        UserGui.SystemStatusEditField.Value = 'Passport Stamp Complete';
        % Animate Passport Removal
        for i= 1:25
            delete(passport);
            passport = PlaceObject('passport_ply.PLY',[(0.15+i*0.006) 0.22 1.00]);
            drawnow
            pause(0);
        end
        delete(passport);
        UserGui.SystemStatusEditField.Value = 'Enter Next Passport';
    end
    pause(0.1);
end

% Move Robots to Default position for system shutdown.
PSRFunctionsKent.moveTo(QStampStartLift,QVP6Default,VP6robot,UR3robot,UserGui,Prisms);
PSRFunctionsKent.moveTo(QPassportStart,QUR3Default,UR3robot,VP6robot,UserGui,Prisms);
hold on;
end
