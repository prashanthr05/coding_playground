clc
clear
close all

%% initial conditions

Xsimse3 = LieGroups.SE3.identity;
Xsimse23 = LieGroups.SE_2_3.identity;

vse3 = [0; 0; 0];
%% constant
accelerationDueToGravity = [0 0 -9.80665]';
%% motion inputs
freeBodyAccelerationInMixedFrame = [0.01 0.02 0.0]'; % A_pdoubledot_B
angularVelocityInInertialFrame = [0.001 0.01 0.0]'; % A_omega_{A,B}

%% time
t = 0.0;
tEnd = 10;
dt = 0.01;
iter = 1;
time = [];
%% simulate
while (t < tEnd)
    [Rse3, pse3] = LieGroups.SE3.extractSE3(Xsimse3);    
    vse3 = vse3 + freeBodyAccelerationInMixedFrame*dt;    
    use3 = [Rse3'*vse3*dt; Rse3'*angularVelocityInInertialFrame*dt];
    use3exphat = LieGroups.SE3.exphat(use3);
    Xsimse3 = LieGroups.SE3.compose(Xsimse3, use3exphat);
    
    [Rse23, pse23, vse23] = LieGroups.SE_2_3.extractSE23(Xsimse23);
    vse23 = vse23 + freeBodyAccelerationInMixedFrame*dt;
    use23 = [0; 0; 0; Rse23'*angularVelocityInInertialFrame*dt; Rse23'*vse23*dt];
    use23exphat = LieGroups.SE_2_3.exphat(use23);
    Xsimse23 = LieGroups.SE_2_3.compose(Xsimse23, use23exphat);
    
    time = [time t];
    simse3Pos(:, iter) = pse3;
    simse3Vel(:, iter) = vse3;
    simse3RPY(:, iter) = rot2rpy(Rse3);
    
    simse23Pos(:, iter) = pse23;
    simse23Vel(:, iter) = vse23;
    simse23RPY(:, iter) = rot2rpy(Rse23);
    t = t+dt;
    iter = iter +1;
end

%%
figure
plot3(simse3Pos(1, :), simse3Pos(2, :), simse3Pos(3, :), 'r-*', 'LineWidth', 2)
hold on
plot3(simse23Pos(1, :), simse23Pos(2, :), simse23Pos(3, :), 'b-*', 'LineWidth', 2)
legend('se3', 'se23')

%% 
figure
for plotidx = 1:3
    subplot(3, 3, plotidx)
    plot(time, simse3Pos(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(time, simse23Pos(plotidx, :), '--', 'LineWidth', 2)
    legend('se3', 'se23')
    
    subplot(3, 3, plotidx+3)
    plot(time, simse3RPY(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(time, simse23RPY(plotidx, :), '--', 'LineWidth', 2)
    legend('se3', 'se23')
    
    subplot(3, 3, plotidx+6)
    plot(time, simse3Vel(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(time, simse23Vel(plotidx, :), '--', 'LineWidth', 2)
    legend('se3', 'se23')
end