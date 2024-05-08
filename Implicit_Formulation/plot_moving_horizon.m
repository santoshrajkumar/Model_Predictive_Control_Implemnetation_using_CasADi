close all;

figure;
N=5;

subplot(2,1,1)
plot(tspan,X_mpc(1,:),'r','LineWidth',2)
hold on

subplot(2,1,2)
stairs(tspan,[u_mpc nan],'b','LineWidth',2)
hold on


for i=1:(length(tspan)-1)
    subplot(2,1,1)
    h1(i)=stairs((i-1)*params.ctrldT:params.ctrldT:(i-1)*params.ctrldT+params.ctrldT*....
                    params.mpc_horizon,X_pred(1,:,i),'b-.','LineWidth',3);
    h2(i)=xline((i-1)*params.ctrldT);
    drawnow
    % xlim([(i-1)*params.ctrldT (i-1)*params.ctrldT+params.ctrldT*params.mpc_horizon])
    legend('Actual', 'Predicted','Horizon')
    ylabel('\theta')
    xlabel('Time(s)')
    subplot(2,1,2)
    h3(i)= stairs((i-1)*params.ctrldT:params.ctrldT:(i-1)*params.ctrldT+params.ctrldT*....
                    params.mpc_horizon,[U_pred(:,:,i) nan],'k-.','LineWidth',2);
    h4(i)=xline((i-1)*params.ctrldT);
    drawnow
    legend('Actual', 'Predicted','Horizon')
    % xlim([(i-1)*params.ctrldT (i-1)*params.ctrldT+params.ctrldT*params.mpc_horizon])
    ylabel('u')
    xlabel('Time(s)')

    pause(0.1);
    delete(h1(i))
    delete(h2(i))
    delete(h3(i))
    delete(h4(i))
end
