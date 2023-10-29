function Environment()

hold on

%Set up workspace
axis equal;  
xlabel ('X');
ylabel ('Y');
zlabel ('Z');
grid on;

PlaceObject('fullroom_ply.ply',[0.175 0.1 1]);

camlight
end


