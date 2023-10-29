

linksL = [
    Revolute('d',0,'a',0.1005,'alpha',0,'offset',0,'qlim', [-pi pi]);
    Revolute('d',0,'a',0.0455,'alpha',0,'offset',0,'qlim', [-pi pi]);
    ];
linksR = [
    Revolute('d',0,'a',-0.1005,'alpha',0,'offset',0,'qlim', [-pi pi]);
    Revolute('d',0,'a',-0.0455,'alpha',0,'offset',0,'qlim', [-pi pi]);
    ];
left = SerialLink(linksL, 'name', 'left');
right = SerialLink(linksR, 'name', 'right');
hold on;
