clc
clear
close all

%% load Cpp
fileName = 'se3Out.txt';
fileID = fopen(fileName, 'r');
formatString = [];
for fi = 1:18
    formatString = [formatString '%f '];
end
formatString = [formatString '%f'];
% formatString = '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
sizeA = [19 Inf];
outCell = fscanf(fileID,formatString, sizeA);
fclose(fileID);

timeCpp = outCell(1, :);
simPosCpp = outCell(2:4, :);
simRPYCpp = outCell(5:7, :);

unFiltPosCpp = outCell(8:10, :);
unFiltRPYCpp = outCell(11:13, :);

estPosCpp = outCell(14:16, :);
estRPYCpp = outCell(17:19, :);

%% landmarks
b0 = [2; 0; 0];
b1 = [3; -1; -1];
b2 = [2; -1; 1];
b3 = [2; 1; 1];
b4 = [2; 1; -1];

landmarks = [b0 b1 b2 b3 b4];


%% doesnt change through experiment
uNom = [0.1; 0.0; 0.0; 0.0; 0.0; 0.05];
u_sigmas = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
U = diag(u_sigmas.^2);

y_sigmas = [0.01; 0.01; 0.01];
R = diag(y_sigmas.^2); 

%% initial conditions
t = 0.0;
tEnd = 10;
Xsim = LieGroups.SE3.identity;
Xest = LieGroups.SE3.identity;
Xunfilt = LieGroups.SE3.identity;
[R0, p0] = LieGroups.SE3.extractSE3(Xsim);

P = diag([0.0;0.0;0.0;0.0;0.0;0.0]);

%% Update Loop
time = [];
for t = 1:10
    %% Simulation
    % simulate noise
    uNoise = u_sigmas.*unifrnd(-1,1, [6, 1]);
    uSim = uNom;    
    uUnfilt = uNom + uNoise;
    uEst = uNom + uNoise;
           
    % first we move
    uSimExpHat = LieGroups.SE3.exphat(uSim);    
    Xsim = LieGroups.SE3.compose(Xsim, uSimExpHat);
    [RSimOut, pSimOut] = LieGroups.SE3.extractSE3(Xsim);
    
    % measure landmarks
    measurements = []; 
    for lidx = 1:length(landmarks)
        b = landmarks(:, lidx);
        yNoise = y_sigmas.*unifrnd(-1,1, [3, 1]);
        yLandmark = LieGroups.SE3.act(LieGroups.SE3.inverse(Xsim), b); % landmark measurement, before adding noise
        yLandmark = yLandmark + yNoise;
        measurements = [measurements yLandmark]; % store for estimator below
    end
    
    %% Estimation
    [REst, pEst] = LieGroups.SE3.extractSE3(Xest);
    % first we move
    [uEstExpHat, J_x, J_u] = LieGroups.SE3.exphat(uEst);    
    Xest = LieGroups.SE3.compose(Xest, uEstExpHat);
    
    P = (J_x * P *J_x') + (J_u * U * J_u');
    
    %correct using measurement of each link
    for lidx = 1:length(landmarks)
        b = landmarks(:, lidx);
        y = measurements(:, lidx);
        
        [Xest_inv, J_xi_x] = LieGroups.SE3.inverse(Xest);
        [e, J_e_xi] = LieGroups.SE3.act(Xest_inv, b);
        H = J_e_xi*J_xi_x;
        E = H*P*H';
        
        z = y - e;
        Z = E + R;
        
        K = P*(H')*inv(Z);        
        dx = K*z;
        
        correction = LieGroups.SE3.exphat(dx);
        Xest = LieGroups.SE3.compose(Xest, correction);
        P = P - K*Z*K';
    end
    
    [REstOut, pEstOut] = LieGroups.SE3.extractSE3(Xest);
    
    %% Unfiltered
    uUnfiltExpHat = LieGroups.SE3.exphat(uUnfilt);    
    Xunfilt = LieGroups.SE3.compose(Xunfilt, uUnfiltExpHat);
    [RUnfiltOut, pUnfiltOut] = LieGroups.SE3.extractSE3(Xunfilt);
        
    % log simulated
    time = [time; t-1];
    
    simPos(:, t) = pSimOut;
    simRPY(:, t) = rot2rpy(RSimOut);
    
    estPos(:, t) = pEstOut;
    estRPY(:, t) = rot2rpy(REstOut);
        
    unFiltPos(:, t) = pUnfiltOut;
    unFiltRPY(:, t) = rot2rpy(RUnfiltOut);
    

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
plot3(simPosCpp(1, :), simPosCpp(2, :), simPosCpp(3, :), '-*', 'LineWidth', 2)
hold on
plot3(unFiltPos(1, :), unFiltPos(2, :), unFiltPos(3, :), 'g-x', 'LineWidth', 2)
hold on
plot3(unFiltPosCpp(1, :), unFiltPosCpp(2, :), unFiltPosCpp(3, :), '-x', 'LineWidth', 2)
hold on
plot3(estPos(1, :), estPos(2, :), estPos(3, :), 'b-o', 'LineWidth', 2)
hold on
plot3(estPosCpp(1, :), estPosCpp(2, :), estPosCpp(3, :), '-o', 'LineWidth', 2)
legend('Initial Position', 'B0', 'B1', 'B2', 'B3', 'B4', 'sim-matlab', 'sim-cpp', 'unfilt-matlab', 'unfilt-cpp','est-matlab', 'est-cpp')


%% matlab vs cpp

figure
for plotidx = 1:3
    subplot(3, 2, 2*plotidx-1)
    plot(time, simPos(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(timeCpp, simPosCpp(plotidx, :), ':', 'LineWidth', 2)
    hold on
    plot(time, unFiltPos(plotidx, :), '-x', 'LineWidth', 2)
    hold on
    plot(time, unFiltPosCpp(plotidx, :), '-x', 'LineWidth', 2)
    hold on
    plot(time, estPos(plotidx, :), 'o', 'LineWidth', 2)
    hold on
    plot(time, estPosCpp(plotidx, :), 'o', 'LineWidth', 2)
    legend('sim-matlab', 'sim-cpp', 'unfilt-matlab', 'unfilt-cpp','est-matlab', 'est-cpp')
    xlabel('Time(s)', 'FontSize', 24)
    ylabel('Position in m', 'FontSize', 24)
    
    subplot(3, 2, 2*plotidx)
    plot(time, simRPY(plotidx, :), '--', 'LineWidth', 2)
    hold on
    plot(timeCpp, simRPYCpp(plotidx, :), ':', 'LineWidth', 2)
    hold on
    plot(time, unFiltRPY(plotidx, :), '-x', 'LineWidth', 2)
    hold on
    plot(time, unFiltRPYCpp(plotidx, :), '-x', 'LineWidth', 2)
    hold on
    plot(time, estRPY(plotidx, :), 'o', 'LineWidth', 2)
    hold on
    plot(time, estRPYCpp(plotidx, :), 'o', 'LineWidth', 2)
    legend('sim-matlab', 'sim-cpp', 'unfilt-matlab', 'unfilt-cpp','est-matlab', 'est-cpp')
    xlabel('Time(s)', 'FontSize', 24)
    ylabel('RPY in radians', 'FontSize', 24)
end