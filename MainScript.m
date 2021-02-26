%% Setup workspace
clear;
clc;
format long g;

%% Define system parameters
% Global
Global.g = 9.81;

% Workspace
Work.dx = 0.5;
Work.W = 1;
Work.L = 2;

% Robot
Robot.alpha = 0.8;
Robot.L1 = sqrt((Work.dx + Work.W)^2 + (Work.L/2)^2)/(1 + Robot.alpha);
Robot.L2 = Robot.alpha*Robot.L1;
Robot.m = 1;

%% NN
NumRuns = 500;
[TrainingInputData,TrainingOutputData,~] = GenerateData(NumRuns,0,Global,Work,Robot);
netconf = [20];
net = feedforwardnet(netconf);
net.trainFcn = 'trainlm';
net.trainParam.epochs = 2000;
net = train(net,TrainingInputData,TrainingOutputData);

figure;
subplot(4,1,1);
stairs(TrainingInputData(1,:),'LineWidth',1);
hold on;
stairs(TrainingInputData(2,:),'LineWidth',1);
stairs(TrainingInputData(3,:),'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('Angles (deg)','Interpreter','latex');
legend('$\gamma$','$\theta_1$','$\theta_2$','Interpreter','latex');
PlotFormat();

subplot(4,1,2);
stairs(TrainingInputData(4,:),'LineWidth',1);
hold on;
stairs(TrainingInputData(5,:),'LineWidth',1);
stairs(TrainingInputData(6,:),'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('Angular velocities (deg/s)','Interpreter','latex');
PlotFormat();

subplot(4,1,3);
stairs(TrainingInputData(7,:),'LineWidth',1);
hold on;
stairs(TrainingInputData(8,:),'LineWidth',1);
stairs(TrainingInputData(9,:),'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('Angular accelerations (deg/s$^2$)','Interpreter','latex');
PlotFormat();

subplot(4,1,4);
stairs(TrainingOutputData(1,:)/1000,'LineWidth',1);
hold on;
stairs(TrainingOutputData(2,:)/1000,'LineWidth',1);
stairs(TrainingOutputData(3,:)/1000,'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('Torques (kN$\cdot$m)','Interpreter','latex');
PlotFormat();

%% 
NumRuns = 400;
[ValidationInputData,ValidationOutputData,FaultVec] = GenerateData(NumRuns,0.05,Global,Work,Robot);
PredictedOutputData = net(ValidationInputData);
T_gamma = ValidationOutputData(1,:);
T_theta1 = ValidationOutputData(2,:);
T_theta2 = ValidationOutputData(3,:);
T_gamma_NN = PredictedOutputData(1,:);
T_theta1_NN = PredictedOutputData(2,:);
T_theta2_NN = PredictedOutputData(3,:);

%
figure;
subplot(3,1,1);
stairs(T_gamma/1000,'LineWidth',1);
hold on;
stairs(T_gamma_NN/1000,'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('$T_\gamma$ (kN$\cdot$m)','Interpreter','latex');
legend('Measured','Neural network','Interpreter','latex');
PlotFormat();

subplot(3,1,2);
stairs(T_theta1/1000,'LineWidth',1);
hold on;
stairs(T_theta1_NN/1000,'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('$T_{\theta_1}$ (kN$\cdot$m)','Interpreter','latex');
PlotFormat();

subplot(3,1,3);
stairs(T_theta2/1000,'LineWidth',1);
hold on;
stairs(T_theta2_NN/1000,'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('$T_{\theta_2}$ (kN$\cdot$m)','Interpreter','latex');
PlotFormat();

% 
N = 20;
RMSE_gamma = zeros(1,NumRuns);
RMSE_theta1 = zeros(1,NumRuns);
RMSE_theta2 = zeros(1,NumRuns);
for RunNum = 1:NumRuns
    E = (T_gamma_NN((RunNum - 1)*N + 1:RunNum*N) - T_gamma((RunNum - 1)*N + 1:RunNum*N))/1000;
    RMSE_gamma(RunNum) = sqrt((E*E')/length(E));
    E = (T_theta1_NN((RunNum - 1)*N + 1:RunNum*N) - T_theta1((RunNum - 1)*N + 1:RunNum*N))/1000;
    RMSE_theta1(RunNum) = sqrt((E*E')/length(E));
    E = (T_theta2_NN((RunNum - 1)*N + 1:RunNum*N) - T_theta2((RunNum - 1)*N + 1:RunNum*N))/1000;
    RMSE_theta2(RunNum) = sqrt((E*E')/length(E));
end

%
figure;
stairs(RMSE_gamma,'LineWidth',1);
hold on;
stairs(RMSE_theta1,'LineWidth',1);
stairs(RMSE_theta2,'LineWidth',1);
hold off;
xlabel('Run number','Interpreter','latex');
ylabel('RMSE (kN$\cdot$m)','Interpreter','latex');
legend('$T_\gamma$','$T_{\theta_1}$','$T_{\theta_2}$','Interpreter','latex');
PlotFormat();

%% Fault detection
Tol = (0:0.01:1);
ScoreVec_gamma = zeros(1,length(Tol));
ScoreVec_theta1 = zeros(1,length(Tol));
ScoreVec_theta2 = zeros(1,length(Tol));
for TolNum = 1:length(Tol)
    NumCorrGuess_gamma = 0;
    NumCorrGuess_theta1 = 0;
    NumCorrGuess_theta2 = 0;
    for RunNum = 1:NumRuns
        if RMSE_gamma(RunNum) > Tol(TolNum)
            if FaultVec(1,RunNum) > 0
                NumCorrGuess_gamma = NumCorrGuess_gamma + 1;
            end
        end
        if RMSE_theta1(RunNum) > Tol(TolNum)
            if FaultVec(2,RunNum) > 0
                NumCorrGuess_theta1 = NumCorrGuess_theta1 + 1;
            end
        end
        if RMSE_theta2(RunNum) > Tol(TolNum)
            if FaultVec(3,RunNum) > 0
                NumCorrGuess_theta2 = NumCorrGuess_theta2 + 1;
            end
        end
    end
    ScoreVec_gamma(TolNum) = NumCorrGuess_gamma/sum(FaultVec(1,:))*100;
    ScoreVec_theta1(TolNum) = NumCorrGuess_theta1/sum(FaultVec(2,:))*100;
    ScoreVec_theta2(TolNum) = NumCorrGuess_theta2/sum(FaultVec(3,:))*100;
end

%
figure;
subplot(2,1,1);
stairs(Tol,ScoreVec_gamma,'LineWidth',1);
hold on;
stairs(Tol,ScoreVec_theta1,'LineWidth',1);
stairs(Tol,ScoreVec_theta2,'LineWidth',1);
hold off;
xlabel('Fault detection trigger RMSE (kN$\cdot$m)','Interpreter','latex');
ylabel('Fault detection success rate (\%)','Interpreter','latex');
legend('$T_\gamma$','$T_{\theta_1}$','$T_{\theta_2}$','Interpreter','latex');
PlotFormat();

% False alarms
Tol = (0:0.01:1);
ScoreVec_gamma = zeros(1,length(Tol));
ScoreVec_theta1 = zeros(1,length(Tol));
ScoreVec_theta2 = zeros(1,length(Tol));
for TolNum = 1:length(Tol)
    NumCorrGuess_gamma = 0;
    NumCorrGuess_theta1 = 0;
    NumCorrGuess_theta2 = 0;
    for RunNum = 1:NumRuns
        if RMSE_gamma(RunNum) <= Tol(TolNum)
            if FaultVec(1,RunNum) < 1
                NumCorrGuess_gamma = NumCorrGuess_gamma + 1;
            end
        end
        if RMSE_theta1(RunNum) <= Tol(TolNum)
            if FaultVec(2,RunNum) < 1
                NumCorrGuess_theta1 = NumCorrGuess_theta1 + 1;
            end
        end
        if RMSE_theta2(RunNum) <= Tol(TolNum)
            if FaultVec(3,RunNum) < 1
                NumCorrGuess_theta2 = NumCorrGuess_theta2 + 1;
            end
        end
    end
    ScoreVec_gamma(TolNum) = 100 - NumCorrGuess_gamma/(NumRuns - sum(FaultVec(1,:)))*100;
    ScoreVec_theta1(TolNum) = 100 - NumCorrGuess_theta1/(NumRuns - sum(FaultVec(2,:)))*100;
    ScoreVec_theta2(TolNum) = 100 - NumCorrGuess_theta2/(NumRuns - sum(FaultVec(3,:)))*100;
end

%
subplot(2,1,2);
stairs(Tol,ScoreVec_gamma,'LineWidth',1);
hold on;
stairs(Tol,ScoreVec_theta1,'LineWidth',1);
stairs(Tol,ScoreVec_theta2,'LineWidth',1);
hold off;
xlabel('Fault detection trigger RMSE (kN$\cdot$m)','Interpreter','latex');
ylabel('False alarm rate (\%)','Interpreter','latex');
% legend('$T_\gamma$','$T_{\theta_1}$','$T_{\theta_2}$','Interpreter','latex');
PlotFormat();