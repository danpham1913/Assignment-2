linksL = [
    Revolute('d',0,'a',0.05,'alpha',0,'offset',0,'qlim', [-pi pi]);
    Revolute('d',0,'a',0.05,'alpha',0,'offset',0,'qlim', [-pi pi]);
    ];
linksR = [
    Revolute('d',0,'a',-0.05,'alpha',0,'offset',0,'qlim', [-pi pi]);
    Revolute('d',0,'a',-0.05,'alpha',0,'offset',0,'qlim', [-pi pi]);
    ];
left = SerialLink(linksL, 'name', 'LEFT');
right = SerialLink(linksR, 'name', 'RIGHT');

hold on

 
left.base = transl(0,0,0)*rpy2tr(0, 0, deg2rad(90), 'xyz');
right.base = transl(0,0,0)*rpy2tr(0, 0, deg2rad(90), 'xyz');


 

%jointvalues

q0L = [deg2rad(-30),deg2rad(-45)]; %opens gripper left
q0R = [deg2rad(30),deg2rad(45)]; %opens gripper right
qCL = [deg2rad(-45),deg2rad(-45)]; %close gripper left
qCR = [deg2rad(45),deg2rad(45)]; %close gripper right

