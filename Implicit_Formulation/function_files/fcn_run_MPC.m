%{

Author: Santosh Rajkumar
Current Affiiliation: The Ohio State University


%% This function runs at each sampling instant to compute the optimal control

inputs:
    params: various parameters as explained below
    x0: measured system output at each time instant

Outputs:
    u: The optimal control to be applied
    cost: the value of the cost function at the optimal decision variables
    u_val:the control signal matrix  over the horizon
    x_val:the state matrix over the horizon

%}

function [u,cost,u_val,x_val] = fcn_run_MPC(params,x0)


N=params.mpc_horizon; % divisions in the time horizon for the mpc
dT=params.ctrldT; % time interval between any two divisions of the horizon
xd=params.xd; % desired states
nctrl = params.nctrl; % number of controls
nstates=params.nstates; % number of states

Q_x=params.Q_x; % the Q matrix in the cost function
R_u=params.R_u; % the R matrix in the cost function


% initialize the NLP problem
opti = casadi.Opti(); 

%initialize the state matrix over the time horizon
x=opti.variable(nstates,N+1);

% initialize the control matrix over the time horizon
U=opti.variable(nctrl,N);

% compute objective function/cost function
obj=0;
for k=1:N
    xk = x(:,k); uc=U(:,k);
    obj = obj+(xk-xd(:,k))'*Q_x*(xk-xd(:,k))+uc'*R_u*uc;
end

% cast the NLP as a minimization problem
opti.minimize(obj);

% load the RHS/ f of the dynamics of the system
odef = @(x,u) fcn_invpend_spring_ode_rhs(x,u);

% compute the dynamics using discrete RK-4 integration
for k=1:N
    xk1 = runge_kutta_4(odef,dT,x(:,k),U(:,k));
    opti.subject_to(x(:,k+1)==xk1); %  impose the continuity constraint
end



% impose the remaining equality constraint
opti.subject_to(x(:,1)==x0);

% impose constraints on the states
opti.subject_to(params.xmin(1) <= x(1,:) <= params.xmax(1));
% impose constraints on the controls
opti.subject_to(params.umin <= U <=params.umax);

% set options for the IPOPT solver
opts=struct('ipopt', struct('print_level', 0),'print_time',0);

% ---- initial values for the decision variables ---
opti.set_initial(x, params.xinit);
opti.set_initial(U, params.uinit');

% ---- solve NLP ------
opti.solver('ipopt',opts); % set numerical backend as the IPOPT solver
sol = opti.solve();   % obtain solution using the IPOPT solver

u = sol.value(U(:,1)); % the control signal at t_1 to be used
u_val = sol.value(U); % obtain the predicted control matrix over the horizon
x_val = sol.value(x); % obtain the predicted state matrix over the horizon
cost = sol.value(obj); % obtain the cost function value

end
