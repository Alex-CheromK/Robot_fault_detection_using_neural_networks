% This main script tests a basic RMSE-based fault detection algorithm for
% identifying faulty motor operation in a robot manipulator. The algorithm
% uses neural networks to make predictions of expected motor torques, and
% if these values diverge significantly from measured values, then a fault
% is triggered.

%% Setup workspace
clear;
clc;
format long g;

%% Define system parameters
% Gravitational acceleration (m/s^2)
Global.g = 9.81;

% Spacing between workbenches (m)
Work.dx = 0.5;
% Workbench width (m)
Work.W = 1;
% Workbench length (m)
Work.L = 2;

% Robot arm length ratio
Robot.alpha = 0.8;
% Robot arm lengths (m)
Robot.L1 = sqrt((Work.dx + Work.W)^2 + (Work.L/2)^2)/(1 + Robot.alpha);
Robot.L2 = Robot.alpha*Robot.L1;
% Note: Above lengths are computed such that the total robot arm length is
% long enough to cover both workbench surfaces.

% Mass being carried by the robot (kg)
Robot.m = 1;

%% Train neural network
% Define the number of training runs, where a single run involves the robot
% transferring a package from one location to another
NumRuns = 500;

% Generate training data by simulating the robot's movement and storing
% link angles and motor torques
[TrainingInputData,TrainingOutputData,~] =...
    GenerateData(NumRuns,0,Global,Work,Robot);

% Create a feedforward neural network with a single hidden layer consisting
% of 20 interneurons
netconf = [20];
net = feedforwardnet(netconf);
% Use the Levenberg-Marquardt backpropagation algorithm for training
net.trainFcn = 'trainlm';
% Specify the maximum number of epochs to use for training
net.trainParam.epochs = 2000;
% Train the neural network
net = train(net,TrainingInputData,TrainingOutputData);

% Plot all input training data (i.e., link angles, angular velocities, and
% angular accelerations) and output training data (i.e., motor torques).
% Link angles
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

% Link angular velocities
subplot(4,1,2);
stairs(TrainingInputData(4,:),'LineWidth',1);
hold on;
stairs(TrainingInputData(5,:),'LineWidth',1);
stairs(TrainingInputData(6,:),'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('Angular velocities (deg/s)','Interpreter','latex');
PlotFormat();

% Link angular accelerations
subplot(4,1,3);
stairs(TrainingInputData(7,:),'LineWidth',1);
hold on;
stairs(TrainingInputData(8,:),'LineWidth',1);
stairs(TrainingInputData(9,:),'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('Angular accelerations (deg/s$^2$)','Interpreter','latex');
PlotFormat();

% Motor torques
subplot(4,1,4);
stairs(TrainingOutputData(1,:)/1000,'LineWidth',1);
hold on;
stairs(TrainingOutputData(2,:)/1000,'LineWidth',1);
stairs(TrainingOutputData(3,:)/1000,'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('Torques (kN$\cdot$m)','Interpreter','latex');
PlotFormat();

%% Check neural network performance
% Specify the number of runs used to generate new validation data
NumRuns = 400;

% Generate validation input and output data
[ValidationInputData,ValidationOutputData,FaultVec] =...
    GenerateData(NumRuns,0.05,Global,Work,Robot);
% Compute neural network predictions using the validation input data
PredictedOutputData = net(ValidationInputData);

% Store all data into appropriately named variables to visually enhance
% coding.
% Actual validation output data
T_gamma = ValidationOutputData(1,:);
T_theta1 = ValidationOutputData(2,:);
T_theta2 = ValidationOutputData(3,:);
% Output data predicted by the neural network
T_gamma_NN = PredictedOutputData(1,:);
T_theta1_NN = PredictedOutputData(2,:);
T_theta2_NN = PredictedOutputData(3,:);

% Plot comparison of validation output data and neural network predictions.
% Yaw motor torque
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

% Link 1 motor torque
subplot(3,1,2);
stairs(T_theta1/1000,'LineWidth',1);
hold on;
stairs(T_theta1_NN/1000,'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('$T_{\theta_1}$ (kN$\cdot$m)','Interpreter','latex');
PlotFormat();

% Link 2 motor torque
subplot(3,1,3);
stairs(T_theta2/1000,'LineWidth',1);
hold on;
stairs(T_theta2_NN/1000,'LineWidth',1);
hold off;
xlabel('Time-step number','Interpreter','latex');
ylabel('$T_{\theta_2}$ (kN$\cdot$m)','Interpreter','latex');
PlotFormat();

% Compute root-mean-square-error (RMSE) between actual validation output
%  data and neural network predictions.
% Number of time-steps within a single run (must match value in
% "GenerateData()" function
N = 20;

% Initialize arrays that will contain RMSE values corresponding to each
% individual run
RMSE_gamma = zeros(1,NumRuns);
RMSE_theta1 = zeros(1,NumRuns);
RMSE_theta2 = zeros(1,NumRuns);
% Loop through all runs and compute the corresponding RMSE value for each
% run
for RunNum = 1:NumRuns
    E = (T_gamma_NN((RunNum - 1)*N + 1:RunNum*N)...
        - T_gamma((RunNum - 1)*N + 1:RunNum*N))/1000;
    RMSE_gamma(RunNum) = sqrt((E*E')/length(E));
    E = (T_theta1_NN((RunNum - 1)*N + 1:RunNum*N)...
        - T_theta1((RunNum - 1)*N + 1:RunNum*N))/1000;
    RMSE_theta1(RunNum) = sqrt((E*E')/length(E));
    E = (T_theta2_NN((RunNum - 1)*N + 1:RunNum*N)...
        - T_theta2((RunNum - 1)*N + 1:RunNum*N))/1000;
    RMSE_theta2(RunNum) = sqrt((E*E')/length(E));
end

% Plot RMSE values over the run number for each motor torque
figure;
stairs(RMSE_gamma,'LineWidth',1);
hold on;
stairs(RMSE_theta1,'LineWidth',1);
stairs(RMSE_theta2,'LineWidth',1);
hold off;
xlabel('Run number','Interpreter','latex');
ylabel('RMSE (kN$\cdot$m)','Interpreter','latex');
legend('$T_\gamma$','$T_{\theta_1}$','$T_{\theta_2}$','Interpreter',...
    'latex');
PlotFormat();

%% Implement basic fault detection algorithm
% Define a vector of tolerances that will be used to evaluate the fault
% detection algorithm
Tol = (0:0.01:1);
% Note: If the RMSE value over a particular run exceeds the fault-detection
% algorithms tolerance values, then a fault is identified. To understand
% what tolerance value works best, this code tests the algorithm using a
% range of tolerance values. Hence the vector of tolerance values defined
% above.

% Initialize vectors that will store the performance of the
% fault-detection algorithm correspnding to each tested tolerance value
ScoreVec_gamma = zeros(1,length(Tol));
ScoreVec_theta1 = zeros(1,length(Tol));
ScoreVec_theta2 = zeros(1,length(Tol));

% For each tolerance value, run a basic RMSE fault-detection algorithm
for TolNum = 1:length(Tol)
    % To track the performance of the algorithm, we must count the number
    % of correct fault detections at each motor.
    % The following variables are initialized at zero and are intended to
    % keep track of the number of correct detections.
    NumCorrGuess_gamma = 0;
    NumCorrGuess_theta1 = 0;
    NumCorrGuess_theta2 = 0;
    
    % Loop through all runs from the validation dataset and check for
    % faults
    for RunNum = 1:NumRuns
        % If the current run's RMSE exceeds the tolerance that is currently
        % being tested, the algorithm detects a fault...
        if RMSE_gamma(RunNum) > Tol(TolNum)
            % Note: The variable "FaultVec" below contains the correct
            % diagnosis of faults.
            % If the algorithm has detected a fault, AND a fault acttually
            % exists, the number of correct diagnoses must increase by one
            if FaultVec(1,RunNum) > 0
                NumCorrGuess_gamma = NumCorrGuess_gamma + 1;
            end
        end
        % Repeat the above approach for the motor of Link 1
        if RMSE_theta1(RunNum) > Tol(TolNum)
            if FaultVec(2,RunNum) > 0
                NumCorrGuess_theta1 = NumCorrGuess_theta1 + 1;
            end
        end
        % Repeat the above approach for the motor of Link 2
        if RMSE_theta2(RunNum) > Tol(TolNum)
            if FaultVec(3,RunNum) > 0
                NumCorrGuess_theta2 = NumCorrGuess_theta2 + 1;
            end
        end
    end
    % Compute the final success score for each motor by calculating the
    % percentages of runs that were diagnosed correctly
    ScoreVec_gamma(TolNum) = NumCorrGuess_gamma/sum(FaultVec(1,:))*100;
    ScoreVec_theta1(TolNum) = NumCorrGuess_theta1/sum(FaultVec(2,:))*100;
    ScoreVec_theta2(TolNum) = NumCorrGuess_theta2/sum(FaultVec(3,:))*100;
end

% Plot the success scores corresponding to each tolerance that was tested
figure;
subplot(2,1,1);
stairs(Tol,ScoreVec_gamma,'LineWidth',1);
hold on;
stairs(Tol,ScoreVec_theta1,'LineWidth',1);
stairs(Tol,ScoreVec_theta2,'LineWidth',1);
hold off;
xlabel('Fault detection trigger RMSE (kN$\cdot$m)','Interpreter','latex');
ylabel('Fault detection success rate (\%)','Interpreter','latex');
legend('$T_\gamma$','$T_{\theta_1}$','$T_{\theta_2}$','Interpreter',...
    'latex');
PlotFormat();

% Repeat the above code, however this time, count the number of false
% alarms reported by the algorithm
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
    ScoreVec_gamma(TolNum) =...
        100 - NumCorrGuess_gamma/(NumRuns - sum(FaultVec(1,:)))*100;
    ScoreVec_theta1(TolNum) =...
        100 - NumCorrGuess_theta1/(NumRuns - sum(FaultVec(2,:)))*100;
    ScoreVec_theta2(TolNum) =...
        100 - NumCorrGuess_theta2/(NumRuns - sum(FaultVec(3,:)))*100;
end

% Plot the rate of false alarms corresponding to each tolerance that was
% tested
subplot(2,1,2);
stairs(Tol,ScoreVec_gamma,'LineWidth',1);
hold on;
stairs(Tol,ScoreVec_theta1,'LineWidth',1);
stairs(Tol,ScoreVec_theta2,'LineWidth',1);
hold off;
xlabel('Fault detection trigger RMSE (kN$\cdot$m)','Interpreter','latex');
ylabel('False alarm rate (\%)','Interpreter','latex');
PlotFormat();