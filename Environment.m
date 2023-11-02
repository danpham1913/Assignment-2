function Environment()

hold on

%Set up workspace
axis ([-1 1.3 -1 1 0 2.5]);
axis equal;  
xlabel ('X');
ylabel ('Y');
zlabel ('Z');
grid on;

%Set up walls
% surf([-5,-5;5,5],[-5,5;-5,5],[0.01,0.01;0.01,0.01],'CData',imread('BestConcrete.jpg'),'FaceColor','texturemap');
% surf([-5,5;-5,5],[4,4;4,4],[3,3;0.01,0.01],'CData',imread('ConstructionSite.jpg'),'FaceColor','texturemap');
% surf([-5,-5;-5,-5],[4,-4;4,-4],[3,3;0.01,0.01],'CData',imread('ConstructionSite.jpg'),'FaceColor','texturemap');
% surf([4,4;4,4],[0.9,1.4;0.9,1.4],[1,1;1.5,1.5],'CData',imread('Warning.jpg'),'FaceColor','texturemap');

%Add objects
% PlaceObject('tableBrown2.1x1.4x0.5m.ply',[1 0 0])
% PlaceObject('fenceAssemblyGreenRectangle4x8x2.5m.ply')
% PlaceObject('Worker.ply',[2 3 0])
PlaceObject('personFemaleBusiness.ply',[0.75 0 0]);
 PlaceObject('ColoredRoom.ply',[0 0 1]);
% PlaceObject('stampper_ply.ply',[0 0 0.8])
camlight

