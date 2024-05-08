clc; clear; close all;


addpath function_files

% add path to the CasADi folder

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

%% Run the Simulation 
tsim=20;
% time vector from 0-tsim with with samplting time = 0.1
% the sampling time can differ
tspan = 0:0.1:tsim; 

N = length(tspan)-1; % length of the tspan vector
dt=0.05; % time interval for ode45 integration (as if we have a continous system)

% initial conditions s;
x0=[deg2rad(35); -0.1];

% storage variables
X_mpc=x0; % first is the initial state
u_mpc = []; % optimal control storage
cost_mpc=[]; % cost function storage
X_act=[]; % continuous time system output store
t=[]; % continuous time store (indeed discrete, but finer)

% desired state matrix for the horizon
params.xd = repmat([0;0],1,params.mpc_horizon+1); 

% iterate over each sampling instant
for k=1:N
    x_now = X_mpc(:,end); % current state (as if measured)
    % initialize state matrix (for NLP problem search)
    params.xinit = repmat(x0,1,params.mpc_horizon+1); 
    % initialize control matrix (for NLP problem search)
    params.uinit = zeros(params.nctrl,params.mpc_horizon); 

    main_loop = tic;
    % compute the control & cost and predicted trajectories
    [u,cost,u_pred,x_pred] = fcn_run_MPC(params,x_now);

    main_loop_time = toc(main_loop)

    % shift the optimal decision varibales
    % to be used as the initial guess for the next sampling instant
    params.xinit=[x_pred(:,2:end) x_pred(:,end)];
    params.uinit=[u_pred(:,2:end) u_pred(:,end)];
    
    % store the applied control 
    u_mpc = [u_mpc u];
    % store the cost
    cost_mpc = [cost_mpc cost];
    % select sim time from current to the next sample
    tsim_int = (k-1)*params.ctrldT:dt:k*params.ctrldT;
    % apply the control to the continuous time system
    [tn, xk1] = ode45(@(t,x) fcn_invpend_spring_ode(t,x,u), tsim_int, x_now);
    % store the system output for the next sampling instant
    X_mpc = [X_mpc xk1(end,:)'];
    % store the continous time trajectory
    X_act=[X_act; xk1];
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




