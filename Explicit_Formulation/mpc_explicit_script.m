clc; clear; close all;

addpath function_files

addpath(['/Users/santosh/Library/CloudStorage/' ...
            'OneDrive-TheOhioStateUniversity/Research/'...
                            'casadi-3.6.4-osx64-matlab2018b'])

import casadi.*

%% MPC parameters
% time interval between two divisions of MPC horizon
params.ctrldT = 0.1; 
% # of divisions of the mpc horizon
params.mpc_horizon = 5;

% no. of control for the system
params.nctrl=1; 
% no. of states for the system
params.nstates=2; 

params.Q_x=1000*eye(params.nstates); % Weighting matrix Q for state cost
params.R_u=0.1*eye(params.nctrl); % Weighting matrix R for energy cost
params.umin=-2*ones(params.nctrl,1); % minimum value of control signals
params.umax=2*ones(params.nctrl,1); % maximum value of control signals
% minimum value of states
params.xmin=-diag([deg2rad(40); 0.2])*ones(params.nstates,1); 
% maximum value of states
params.xmax=diag([deg2rad(40); 0.2])*ones(params.nstates,1); 


%% Simulation 
tsim=10;
% time vector from 0-tsim with each control interval
tspan = 0:params.ctrldT:tsim; 
N = length(tspan)-1; % length of the tspan vector
dt=0.01; % time interval for ode45 integration


%% Construct the NLP with Multiple Shooting
X=SX.sym('X', params.nstates,params.mpc_horizon+1);
U=SX.sym('U',params.nctrl,params.mpc_horizon);
P=SX.sym('P',params.nstates+.....
                   (params.mpc_horizon+1)*params.nstates+...
                                params.mpc_horizon*params.nctrl,1);
%{ 
The structure of the parameter vector is:
P=[x_now; X(:); U(:)]
%}

% initialize the cost/objective function
obj=0;
% initialize the constraints (other than state & control constraints)
g=[];
% impose the LHS of the initial condition constraint
g=[g;X(:,1)-P(1:params.nstates,1)];

% load the RHS/ f of the dynamics of the system
odef = @(x,u) fcn_invpend_spring_ode_rhs(x,u);

for k=1:params.mpc_horizon
    xk = X(:,k); uc=U(:,k);
    obj = obj+(xk-P(params.nstates*k+1:params.nstates*k+params.nstates))'....
           *params.Q_x*(xk-P(params.nstates*k+1:params.nstates*k+params.nstates))+....
                                                         uc'*params.R_u*uc;
    xk1 = runge_kutta_4(odef,params.ctrldT,xk,uc);
    g=[g;X(:,k+1)-xk1]; % impose LHS of the continuity constraint
end

% Decision variables for the NLP Problem
W = [X(:); U(:)];
nlp_prob = struct('f', obj, 'x', W, 'g', g, 'p', P); % x is the W here (not to be cofused with state)

% options for the NLP Problem
opts = struct;
opts.ipopt.max_iter = 4000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

% plugin the IPOPT solver
solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

% constraints as arguments
args = struct;
% imposing the equality constraints
args.lbg(1:length(g))=0;
args.ubg(1:length(g))=0;

% imposing the state constraints
args.lbx(1:params.nstates:(params.mpc_horizon+1)*params.nstates,1)=-deg2rad(40);
args.ubx(1:params.nstates:(params.mpc_horizon+1)*params.nstates,1)=deg2rad(40);
args.lbx(2:params.nstates:(params.mpc_horizon+1)*params.nstates,1)=-inf;
args.ubx(2:params.nstates:(params.mpc_horizon+1)*params.nstates,1)=inf;
% imposing the control constraints
args.lbx((params.mpc_horizon+1)*params.nstates+1:length(W),1)=-2;
args.ubx((params.mpc_horizon+1)*params.nstates+1:length(W),1)=2;


%% Run the Simulation

tsim=20;
% time vector from 0-tsim with with samplting time = 0.1
% the sampling time can differ
tspan = 0:0.1:tsim; 

N = length(tspan)-1; % length of the tspan vector
dt=0.05; % time interval for ode45 integration (as if we have a continous system)

% initial conditions s;
x0=[deg2rad(35); -0.1];

% desired states over the horizon
xd= repmat([0;0],1,params.mpc_horizon+1);
% desired control over the horizon (it is zero as we want minimum possible)
ud=zeros(params.nctrl,params.mpc_horizon); 



% storage variables
X_mpc=x0; % first is the initial state
u_mpc = []; % optimal control storage
cost_mpc=[]; % cost function storage
X_act=[]; % continuous time system output store
t=[]; % continuous time store (indeed discrete, but finer)

% initialize (states) decision variables for the NLP
Xinit=repmat(x0,1,params.mpc_horizon+1);
% initialize (control) decision variables for the NLP
Uinit = repmat(zeros(params.nctrl,1),1,params.mpc_horizon);

% iterate over each sampling instant
for k=1:N
    x_now = X_mpc(:,end);  % current state (as if measured)
    args.p=[x_now;xd(:);ud(:)]; % populate parameter vector P
    args.x0=[Xinit(:);Uinit(:)]; % = W_init (initial guess of the decision variables W)
    
    main_loop = tic;

    % solve the NLP
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

    main_loop_time = toc(main_loop)

    % obtain the predicted control actions over the horizon & reshape to
    % get in matrix form
    Upred=reshape(full(sol.x(params.nstates*(params.mpc_horizon+1)+1:end)),....
                                           params.nctrl,params.mpc_horizon);
    % shift the predicted control actions to be used as initial guess for
    % the next time step
    Uinit=[Upred(:,2:end) Upred(:,end)];
    % obtain the predicted states over the horizon & reshape to
    % get in matrix form
    Xpred = reshape(full(sol.x(1:params.nstates*(params.mpc_horizon+1))),....
                                      params.nstates,params.mpc_horizon+1);
    % shift the predicted control actions to be used as initial guess for
    % the next time step
    Xinit=[Xpred(:,2:end) Xpred(:,end)];
    % the optimal control action to be applied currently
    u=Upred(:,1);
    % stire the current optimal control action
    u_mpc=[u_mpc u];
    % select sim time from current to the next sample
    tsim_int = (k-1)*params.ctrldT:dt:k*params.ctrldT;
    % apply the control to the continuous time system
    [tn, Xn] = ode45(@(t,x) fcn_invpend_spring_ode(t,x,u), tsim_int, x_now);
    % store the system output for the next sampling instant
    X_mpc = [X_mpc Xn(end,:)'];
    % store the continous time trajectory
    X_act=[X_act; Xn];
    % store the time vector for continous time 
    t=[t;tn];
end

% save the actual trajectory
save('X_pend.mat','X_act');

%% plotting
f=figure;
subplot(3,1,1)
yline(0,'r--','LineWidth',3)
hold on
plot(t,rad2deg(X_act(:,1)),'LineWidth',3)
xlabel('Time (s)')
ylabel('$\theta^o$','Interpreter','latex')
legend('Reference','Actual')
set(gca,'FontSize',25)
grid

subplot(3,1,2)
plot(t,rad2deg(X_act(:,2)),'LineWidth',3)
xlabel('Time (s)')
ylabel('$\omega$ deg/sec','Interpreter','latex')
set(gca,'FontSize',25)
grid
subplot(3,1,3)
stairs(tspan,[u_mpc nan],'-.','LineWidth',3)
xlabel('Time (s)')
ylabel('$M$ N-m','Interpreter','latex')
set(gca,'FontSize',25)
grid

f.Position=[100 100 1000 800];






