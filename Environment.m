function Environment()

hold on

%Set up workspace
axis ([-2 4 -1.5 1.5 0 3])
axis equal;  
xlabel ('X');
ylabel ('Y');
zlabel ('Z');
grid on;

%Set up walls
surf([-2,-2;4,4],[-1.5,1.5;-1.5,1.5],[0.01,0.01;0.01,0.01],'CData',imread('floor.jpg'),'FaceColor','texturemap');
surf([-2,4;-2,4],[-1.5,-1.5;-1.5,-1.5],[3,3;0.01,0.01],'CData',imread('Screenshot(2).jpg'),'FaceColor','texturemap');
surf([4,4;4,4],[-1.5,1.5;-1.5,1.5],[3,3;0.01,0.01],'CData',imread('Screenshot(3).jpg'),'FaceColor','texturemap');
surf([-0.6,-0.4;-0.6,-0.4],[-0.85,-0.85;-0.85,-0.85],[1,1;1.2,1.2],'CData',imread('Warning.jpg'),'FaceColor','texturemap');

%Add objects
PlaceObject('magnetic_switch.PLY',[0 0 1])
PlaceObject('magnetic_switch_w_leads.PLY',[0 0 1])
PlaceObject('personMaleCasual.ply',[0.85 0 0])
PlaceObject('personFemaleBusiness.ply',[1.5 0 0])
PlaceObject('fullroom2_ply (1).PLY',[0 0 1])
PlaceObject('door.PLY',[0 0 1])
PlaceObject('personMaleOld.ply',[2.25 0 0])
PlaceObject('suitcase.ply',[2.25 0 0])
PlaceObject('suitcase2.ply',[0.85 0.25 0])
PlaceObject('suitcase.ply',[1.5 1 0])
PlaceObject('fireExtinguisher.ply',[0 0.8 0])
PlaceObject('emergencyStopButton.ply',[-0.8 -0.8 1])
camlight

