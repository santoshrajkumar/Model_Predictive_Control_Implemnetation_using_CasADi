
function xdot=fcn_invpend_spring_ode_rhs(x,M)
    
% Parameters;
g=9.81;
l=4;
d=3.5;
m=0.15;
k=0.3;
c=0.05;

xdot=[x(2); -(k*d^2/m/l^2)*x(1)-(c/m/l^2)*d^2*(cos(x(1)))^2*x(2)+....
                                        (g/l)*x(1)+(M/m/l^2)];

end