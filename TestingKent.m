clear all;
close all;
clc;
hold on;
%Environment
Environment

%Robot LinearUR3
VP6robot = densoVP6242(transl(-0.2,-0.4,0.60)*trotz(-pi/2));
UR3robot = UR3(transl(-0.2,0.20,0.70));
UR3robot.model.teach();
%Placing Stamp
stamp = PlaceObject('stampper_ply.PLY',[-0.042 -0.12 0.71]);
StampVerts = get(stamp,'Vertices');


%%
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

%Stamp Positions
QStampStart = VP6robot.model.ikcon(transl(-0.042,-0.12,0.71)*troty(pi),VPInitialGuess);
QStampStartLift = VP6robot.model.ikcon(transl(-0.042,-0.12,0.76) * troty(pi),VPInitialGuess);
QStampEndLift = VP6robot.model.ikcon(transl(-0.29,-0.05,0.76) * troty(pi),VPInitialGuess);
QStampEnd = VP6robot.model.ikcon(transl(-0.29,-0.05,0.72)* troty(pi),VPInitialGuess);

%Move Robots to operational position for system startup
input('Press enter to Load New Passport')
[passport,PassportVerts] = loadPassport;
passportLoaded = 1;
Startup = 1;
while passportLoaded == 1
    if Startup == 1;
        [MoveVP0,CurrentStep] = moveTo(QVP6Default,QStampStartLift,VP6robot);
        [MoveUR0,CurrentStep] = moveTo(QUR3Default,QPassportStart,UR3robot);
        Startup=0;
    end
    [MoveUR1,CurrentStep,PassportVerts] = moveTo(QPassportStart,QPassportLift,UR3robot,passport,PassportVerts);
    [MoveUR2,CurrentStep,PassportVerts] = moveTo(QPassportLift,QPassportEnd,UR3robot,passport,PassportVerts);
    [MoveUR2,CurrentStep] = moveTo(QPassportEnd,QPassportLift,UR3robot);
    [moveVP1,CurrentStep] = moveTo(QStampStartLift,QStampStart,VP6robot);
    [moveVP2,CurrentStep,StampVerts] = moveTo(QStampStart,QStampStartLift,VP6robot,stamp,StampVerts);
    [moveVP3,CurrentStep,StampVerts] = moveTo(QStampStartLift,QStampEndLift,VP6robot,stamp,StampVerts);
    [moveVP4,CurrentStep,StampVerts] = moveTo(QStampEndLift,QStampEnd,VP6robot,stamp,StampVerts);
    [moveVP5,CurrentStep,StampVerts] = moveTo(QStampEnd,QStampEndLift,VP6robot,stamp,StampVerts);
    [moveVP6,CurrentStep,StampVerts] = moveTo(QStampEndLift,QStampStartLift,VP6robot,stamp,StampVerts);
    [moveVP7,CurrentStep,StampVerts] = moveTo(QStampStartLift,QStampStart,VP6robot,stamp,StampVerts);
    [moveVP8,CurrentStep] = moveTo(QStampStart,QStampStartLift,VP6robot);
    [MoveUR2,CurrentStep] = moveTo(QPassportLift,QPassportEnd,UR3robot);
    [MoveUR2,CurrentStep,PassportVerts] = moveTo(QPassportEnd,QPassportStart,UR3robot,passport,PassportVerts);

    % Remove Passport
    for i= 1:25
        delete(passport);
        passport = PlaceObject('passport_ply.PLY',[(0.15+i*0.006) 0.22 1.00]);
        drawnow
        pause(0.1);
    end
    delete(passport);

    EnterPassportPrompt = "Load Passport? Y/N: ";
    txt = input(EnterPassportPrompt,"s");
    if isempty(txt)
        txt = 'Y';
    end
    if txt == 'y' || txt == 'Y'
        [passport,PassportVerts] = loadPassport;
        passportLoaded = 1;
    else
        passportLoaded = 0;
        break;
    end
end

function [trajectory,currentStep,ObjectVerts] = moveTo(QStart,QEnd,Robot,Object,ObjectVerts)
% Check if Object is provided
if nargin > 4
    trfix = inv(Robot.model.fkine(Robot.model.getpos).T) * [ObjectVerts, ones(size(ObjectVerts,1),1)]';
    ObjectAttached = 1;
else
    ObjectAttached = 0;
end

%Determine Steps required based on distance
Start = Robot.model.fkineUTS(QStart);
End = Robot.model.fkineUTS(QEnd);
StartTr = Start(1:3,4);
EndTr = End(1:3,4);
travelDistance = norm(StartTr-EndTr);
steps = round(100*travelDistance+15);

%Generate Trajectory
trajectory = jtraj(QStart,QEnd,steps);

% animate trajectory
for i = 1:size(trajectory,1)
    Robot.model.animate(trajectory(i,:));
    if ObjectAttached == 1
        UpdateObject = Robot.model.fkine(trajectory(i,:)).T * trfix;
        trvert = UpdateObject(1:3,:)';
        set(Object,'Vertices',trvert);
    end

    currentStep = i;
    drawnow;
    pause(0);
end

%Return objectvertices so they are updated when called
if ObjectAttached == 1
    ObjectVerts = get(Object,'Vertices');
end
end

function [passport, PassportVerts] = loadPassport()
passport = PlaceObject('passport_ply.PLY',[0.25 0.22 1.00]);

steps = 25;
for i= 1:steps
    delete(passport);
    passport = PlaceObject('passport_ply.PLY',[(0.25-i*0.004) 0.22 1.00]);
    drawnow;
    pause(0);
end
PassportVerts = get(passport,'Vertices');
end



