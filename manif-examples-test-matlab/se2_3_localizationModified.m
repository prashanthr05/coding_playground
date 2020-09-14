clc
clear
close all
%% options
plotCpp = false;
%% load Cpp
fileName = 'se23Out.txt';
fileID = fopen(fileName, 'r');
formatString = [];
for fi = 1:27
    formatString = [formatString '%f '];
end
formatString = [formatString '%f'];

% formatString = '%f %f %f %f %f %f %f %f %f %f';
sizeA = [28 Inf];
outCell = fscanf(fileID,formatString, sizeA);
fclose(fileID);

timeCpp = outCell(1, :);
simPosCpp = outCell(2:4, :);
simRPYCpp = outCell(5:7, :);
simVelCpp = outCell(8:10, :);

unFiltPosCpp = outCell(11:13, :);
unFiltRPYCpp = outCell(14:16, :);
unFiltVelCpp = outCell(17:19, :);

estPosCpp = outCell(20:22, :);
estRPYCpp = outCell(23:25, :);
estVelCpp = outCell(26:28, :);

%% landmarks
b0 = [2; 0; 0];
b1 = [3; -1; -1];
b2 = [2; -1; 1];
b3 = [2; 1; 1];
b4 = [2; 1; -1];

landmarks = [b0 b1 b2 b3 b4];


%% constants
g = [0; 0; -9.80665]; % acceleration due to gravity in inertial frame
dt = 0.1;


%% initial conditions
t = 0.0;
tEnd = 10;
Xsim = LieGroups.SE_2_3.identity;
Xest = LieGroups.SE_2_3.identity;
Xunfilt = LieGroups.SE_2_3.identity;
[R0, p0, v0] = LieGroups.SE_2_3.extractSE23(Xsim);

P = diag([0.001;0.001;0.001;0.01;0.01;0.01;0.001;0.001;0.001]);

%% initial measurements
% freeBodyAccelerationInIMUFrame = [0.001; 0.02; 0.05];
freeBodyAccelerationInMixedFrame = [0.1; 0; 0];
angularVelocityInInertialFrame = [0.02; 0.01; 0];


accelerometerMeasurementInIMUFrame = R0'*(freeBodyAccelerationInMixedFrame- g);
gyroscopeMeasurementInIMUFrame = R0'*angularVelocityInInertialFrame;
prevAccelerometerMeasurementInIMUFrame =  accelerometerMeasurementInIMUFrame;
prevGyroscopeMeasurementInIMUFrame = gyroscopeMeasurementInIMUFrame;

u_sigmas = [0.0; 0.0; 0.0; 0.0001; 0.0001; 0.0001; 0.001; 0.001; 0.001];
U = diag(u_sigmas.^2);

y_sigmas = [0.001; 0.001; 0.001];
R = diag(y_sigmas.^2);

%% Update Loop
iter = 1;
time = [];
while (t < tEnd)
    %% Simulation
    [RSim, pSim, vSimInMixedFrame] = LieGroups.SE_2_3.extractSE23(Xsim);
    accSimInBodyFrame = RSim'*freeBodyAccelerationInMixedFrame;
    omegaSimInBodyFrame = RSim'*angularVelocityInInertialFrame;    
    
    %input vector
    uNom = [RSim'*vSimInMixedFrame*dt + 0.5*dt*dt*accSimInBodyFrame;
            omegaSimInBodyFrame*dt;
            accSimInBodyFrame*dt];
    % simulate noise
    uNoise = u_sigmas.*unifrnd(-1,1, [9, 1]);
    uSim = uNom;
    uUnfilt = uNom + uNoise;
            
    % first we move
    uSimExpHat = LieGroups.SE_2_3.exphat(uSim);
    Xsim = LieGroups.SE_2_3.compose(Xsim, uSimExpHat);
    [RSimOut, pSimOut, vSimOut] = LieGroups.SE_2_3.extractSE23(Xsim)
    accSimOutInBodyFrame = RSimOut'*freeBodyAccelerationInMixedFrame
    omegaSimOutInBodyFrame = RSimOut'*angularVelocityInInertialFrame
    
    % then we measure - simulate IMU measurements
%     accelerometerMeasurementInIMUFrame = accSimInBodyFrame - Rsim'*g + uNoise(7:9);
%     gyroscopeMeasurementInIMUFrame = omegaSimInBodyFrame + uNoise(4:6);
    accelerometerMeasurementInIMUFrame = accSimOutInBodyFrame - RSimOut'*g + uNoise(7:9)
    gyroscopeMeasurementInIMUFrame = omegaSimOutInBodyFrame + uNoise(4:6)
    
    % measure landmarks
    measurements = [];
    for lidx = 1:length(landmarks)
        b = landmarks(:, lidx);
        yNoise = y_sigmas.*unifrnd(-1,1, [3, 1]);
        yLandmark = LieGroups.SE_2_3.act(LieGroups.SE_2_3.inverse(Xsim), b); % landmark measurement, before adding noise
        yLandmark = yLandmark + yNoise;
        measurements = [measurements yLandmark]; % store for estimator below
    end
    
    %% Estimation
    [REst, pEst, vEst] = LieGroups.SE_2_3.extractSE23(Xest);
    accEst = prevAccelerometerMeasurementInIMUFrame + (REst'*g)
    omegaEst = prevGyroscopeMeasurementInIMUFrame
    uEst = [REst'*vEst*dt + 0.5*dt*dt*accEst;
            omegaEst*dt;
            accEst*dt];
    
    % first we move
    [uEstExpHat, J_x, J_u] = LieGroups.SE_2_3.exphat(uEst);
    Xest = LieGroups.SE_2_3.compose(Xest, uEstExpHat);
    
%     P = (J_x * P *J_x') + (J_u * U * J_u');
%     
%     %     %correct using measurement of each link
%     for lidx = 1:length(landmarks)
%         b = landmarks(:, lidx);
%         y = measurements(:, lidx);
%         
%         [Xest_inv, J_xi_x] = LieGroups.SE_2_3.inverse(Xest);
%         [e, J_e_xi] = LieGroups.SE_2_3.act(Xest_inv, b);
%         H = J_e_xi*J_xi_x;
%         E = H*P*H';
%         
%         z = y - e;
%         Z = E + R;
%         
%         K = P*H'*inv(Z);
%         dx = K*z;
%         
%         correction = LieGroups.SE_2_3.exphat(dx);
%         Xest = LieGroups.SE_2_3.compose(Xest, correction);
%         P = P - K*Z*K';
%     end
%     
    [REstOut, pEstOut, vEstOut] = LieGroups.SE_2_3.extractSE23(Xest);
    
    %% Unfiltered
    uUnfiltExpHat = LieGroups.SE_2_3.exphat(uUnfilt);
    Xunfilt = LieGroups.SE_2_3.compose(Xunfilt, uUnfiltExpHat);
    [RUnfiltOut, pUnfiltOut, vUnfiltOut] = LieGroups.SE_2_3.extractSE23(Xunfilt);
    %% store and log
    prevAccelerometerMeasurementInIMUFrame = accelerometerMeasurementInIMUFrame;
    prevGyroscopeMeasurementInIMUFrame = gyroscopeMeasurementInIMUFrame;
    
    % log simulated
    time = [time; t];
    
    simPos(:, iter) = pSimOut;
    simVel(:, iter) = vSimOut;
    simRPY(:, iter) = rot2rpy(RSimOut);
    
    estPos(:, iter) = pEstOut;
    estVel(:, iter) = vEstOut;
    estRPY(:, iter) = rot2rpy(REstOut);
    
    unFiltPos(:, iter) = pUnfiltOut;
    unFiltVel(:, iter) = vUnfiltOut;
    unFiltRPY(:, iter) = rot2rpy(RUnfiltOut);
    
    % update iters
    iter = iter+1;
    t = t + dt;
end


%% plot
figure;
h = scatter3(p0(1), p0(2), p0(3),'filled');
h.SizeData = 150;
text(p0(1)+0.1, p0(2)+0.1, p0(3)+0.1, 'p0', 'Fontsize', 20);
hold on
dx = 0.1;
dy = 0.1;
dz = 0.1;
for idx = 1:length(landmarks)
    x = landmarks(1, idx);
    y = landmarks(2, idx);
    z = landmarks(3, idx);
    h = scatter3(x, y, z,'filled');
    h.SizeData = 150;
    text(x+dx, y+dy, z+dz, ['Beacon ', num2str(idx)], 'Fontsize', 20);
    hold on
end

plot3(simPos(1, :), simPos(2, :), simPos(3, :), 'r-*', 'LineWidth', 2)
hold on
plot3(unFiltPos(1, :), unFiltPos(2, :), unFiltPos(3, :), 'g-x', 'LineWidth', 2)
hold on
plot3(estPos(1, :), estPos(2, :), estPos(3, :), 'b-o', 'LineWidth', 2)
hold on
if plotCpp
    plot3(simPosCpp(1, :), simPosCpp(2, :), simPosCpp(3, :), '-*', 'LineWidth', 2)
    hold on
    plot3(unFiltPosCpp(1, :), unFiltPosCpp(2, :), unFiltPosCpp(3, :), '-x', 'LineWidth', 2)
    hold on
    plot3(estPosCpp(1, :), estPosCpp(2, :), estPosCpp(3, :), '-o', 'LineWidth', 2)
    legend('Initial Position', 'B0', 'B1', 'B2', 'B3', 'B4', 'sim-matlab', 'unfilt-matlab', 'est-matlab','sim-cpp', 'unfilt-cpp', 'est-cpp')
else
    legend('Initial Position', 'B0', 'B1', 'B2', 'B3', 'B4', 'sim-matlab', 'unfilt-matlab', 'est-matlab')
end



%% matlab vs cpp

figure
for plotidx = 1:3
    subplot(3, 3, plotidx)
    plot(time, simPos(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(time, unFiltPos(plotidx, :), '-x', 'LineWidth', 2)
    hold on
    plot(time, estPos(plotidx, :), 'o', 'LineWidth', 2)
    hold on
    if plotCpp
        plot(timeCpp, simPosCpp(plotidx, :), ':', 'LineWidth', 2)
        hold on
        plot(time, unFiltPosCpp(plotidx, :), '-x', 'LineWidth', 2)
        hold on
        plot(time, estPosCpp(plotidx, :), 'o', 'LineWidth', 2)
        legend('sim-matlab', 'unfilt-matlab', 'est-matlab','sim-cpp', 'unfilt-cpp', 'est-cpp')
    else
        legend('sim-matlab', 'unfilt-matlab', 'est-matlab')
    end
    
    subplot(3, 3, plotidx+3)
    plot(time, simRPY(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(time, unFiltRPY(plotidx, :), '-x', 'LineWidth', 2)
    hold on
    plot(time, estRPY(plotidx, :), 'o', 'LineWidth', 2)
    hold on
    if plotCpp
        plot(timeCpp, simRPYCpp(plotidx, :), ':', 'LineWidth', 2)
        hold on
        plot(time, unFiltRPYCpp(plotidx, :), '-x', 'LineWidth', 2)
        hold on
        plot(time, estRPYCpp(plotidx, :), 'o', 'LineWidth', 2)
        legend('sim-matlab', 'unfilt-matlab', 'est-matlab','sim-cpp', 'unfilt-cpp', 'est-cpp')
    else
        legend('sim-matlab', 'unfilt-matlab', 'est-matlab')
    end
    
    subplot(3, 3, plotidx+6)
    plot(time, simVel(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(time, unFiltVel(plotidx, :), '-x', 'LineWidth', 2)
    hold on
    plot(time, estVel(plotidx, :), 'o', 'LineWidth', 2)
    hold on
    if plotCpp
        plot(timeCpp, simVelCpp(plotidx, :), ':', 'LineWidth', 2)
        hold on
        plot(time, unFiltVelCpp(plotidx, :), '-x', 'LineWidth', 2)
        hold on
        plot(time, estVelCpp(plotidx, :), 'o', 'LineWidth', 2)
        legend('sim-matlab', 'unfilt-matlab', 'est-matlab','sim-cpp', 'unfilt-cpp', 'est-cpp')
    else
        legend('sim-matlab', 'unfilt-matlab', 'est-matlab')
    end
end