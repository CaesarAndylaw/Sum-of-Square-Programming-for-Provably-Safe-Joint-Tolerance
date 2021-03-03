function [c,ceq] = nonlcon_reform_6d(x,xwall)
    ROBOT = 'GP50';
    robot=robotproperty(ROBOT);
    cpre = ForKine(x, robot.DH, robot.base, robot.cap);
    xpos = cpre(1);
    c = -(xpos - xwall);
    ceq = [];
end