function xk1 = runge_kutta_4(func,dT,x,u)
  k1 = func(x,       u);
  k2 = func(x+dT/2*k1,u);
  k3 = func(x+dT/2*k2,u);
  k4 = func(x+dT*k3,  u);
  xk1 = x + dT/6 * (k1 + 2*k2 + 2*k3 + k4); 
end