function moveArm()
r = UR3;
steps0 = 50;


gripperArm;


qClosingL = jtraj(q0L, qCL, steps0);
qOpeningL = jtraj(qCL, q0L, steps0);
qClosingR = jtraj(q0R, qCR, steps0);
qOpeningR = jtraj(qCR, q0R, steps0);


 left.animate(q0L);
 right.animate(q0R);
 

left.base = r.model.fkine(r.model.getpos()).T * trotx(deg2rad(-90));
right.base = r.model.fkine(r.model.getpos()).T * trotx(deg2rad(-90));


    for i = steps0
        left.animate(qClosingL(i,:));
        right.animate(qClosingR(i,:));
         
        drawnow();
        pause(0);
    end

end



