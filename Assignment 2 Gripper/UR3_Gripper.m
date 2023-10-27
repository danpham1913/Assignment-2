
r = UR3;   

 
gripperArm;
 %Plotting gripper arm
 left.base = transl(0,0,0)*rpy2tr(0, 0, deg2rad(90), 'xyz');
 right.base = transl(0,0,0)*rpy2tr(0, 0, deg2rad(90), 'xyz');
 
        left.plot(q0L, 'noname', 'nowrist');
        right.plot(q0R, 'noname', 'nowrist');
 
 %moving gripper

 left.base = r.model.fkine(r.model.getpos()).T * trotx(deg2rad(-90));
 right.base = r.model.fkine(r.model.getpos()).T * trotx(deg2rad(-90));
  
  left.animate(q0L);
  right.animate(q0R);
 
 
 
 
 
 


