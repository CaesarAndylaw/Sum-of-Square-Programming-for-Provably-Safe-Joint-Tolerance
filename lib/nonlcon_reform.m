function [c,ceq] = nonlcon_reform(x,xwall)
    theta1 = x(1);
    theta2 = x(2);
    xpos = cos(theta1) + cos(theta2);
    c = -(xpos - xwall);
    ceq = [];
end