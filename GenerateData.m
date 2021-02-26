% This function simulates the motion of the robot and measures the motor
% torques that are necessary for delivering this desired motion. The
% robot's link angles, angular velocities, and angular accelerations are
% the input data for the neural network. The robot's motor torques as the
% output data for the neural network.
% This function takes the following parameters as arguments:
% 1. NumRuns - The total number of runs to simulate. Each runs corresponds
% to the robot moving a package from one location to another along the
% workbenches.
% 2. FaultProb - The probability (specified by the user) that a fault will
% occur  in any motor during a particular run.
% 3. Global, Work, and Robot - Class object containing simulation
% parameters.

function [GlobalInputData,GlobalOutputData,FaultVec] =...
    GenerateData(NumRuns,FaultProb,Global,Work,Robot)
% Number of time-steps within a single run (must match value in the main
% script
N = 20;

% Generate a vector to store timestamps at each time-step
TimeVec = linspace(0,5,N);
% Calculate the sampling period of the analysis (sec)
dt = TimeVec(2) - TimeVec(1);

% Initialize vectors that will store link angles (deg)
gamma = zeros(1,N);
theta1 = zeros(1,N);
theta2 = zeros(1,N);

% The robot picks up a package from one point and moves it to another
% randomly generated point along the workbenches.
% An initial start point "rp1" must be specified
rp1 = [Work.dx 0]';

% The current function uses a nonlinear equation solver to compute the link
% angles that are necessary for locating the package at some point in 3D
% space.
% The nonlinear equation solver requires an initial solution guess...
InitVarVec = [0 -45 90]';

% Initialize arrays that will store the input and output data to be used
% for neural network training and validation
GlobalInputData = zeros(9,N*NumRuns);
GlobalOutputData = zeros(3,N*NumRuns);

% Initialize a vector that tracks whether or not a fault has actually
% occured within the current run
FaultVec = zeros(3,NumRuns);

% Initialize a counter that is necessary for array indexing
BaseNum = 0;

% For each run, begin simulating the robot's movement
for i = 1:NumRuns
    % First, let us randomly generate a fault based on the fault
    % probability specified by the user
    % If the fault probability is greater that zero, then the used intends
    % for there to be a fault...
    if FaultProb > 0
        % First initialize the joint damping vector "c" with non-faulty
        % values...
        c = [10 10 10];
        % Then generate a random number with uniform probability, and if
        % this number is less than the fault probability specified by the
        % user, then a fault occurs...
        if rand <= FaultProb
            % The joint damping of the yaw motor is altered to a faulty
            % value
            c(1) = 20;
            % And the fault is tracked
            FaultVec(1,i) = 1;
        end
        % Repeat the above code for the joint of Link 1
        if rand <= FaultProb
            c(2) = 20;
            FaultVec(2,i) = 1;
        end
        % Repeat the above code for the joint of Link 2
        if rand <= FaultProb
            c(3) = 20;
            FaultVec(3,i) = 1;
        end
    % If the fault probability specified by the user is zero, then all
    % joint damping factors are kept at non-faulty values
    else
        c = [10 10 10];
    end
    
    % Randomly generate the package destination
    rp2 = [
        sign(-1 + rand*2)*(Work.dx + rand*Work.W);
        -Work.L/2 + rand*Work.L];
    
    % Define an elliptical trajectory (in local frame) between "rp1" and
    % "rp2"
    a = norm(rp2 - rp1)/2;
    b = 0.5;
    xe = linspace(0,2*a,N);
    ze = b*sqrt(1 - ((xe - a)/a).^2);
    
    % Convert the ellipse to global frame
    u = (rp2 - rp1)/norm(rp2 - rp1);
    rm = [
        rp1 + xe.*u;
        ze];
    
    % With the trajectory stored, the current package destination will
    % serve as the package start point in the next run
    rp1 = rp2;
    
    % For each time-step within the current run, determine the link angles
    % that are necessary for moving the package along the desired
    % elliptical trajectory using a nonlinear equation solver (i.e., a
    % Newton-Raphson secant method solver)
    for j = 1:N
        SolVec = fsolve(...
            @(VarVec) RobotKinematics(VarVec,rm(:,j),Robot),InitVarVec);
        gamma(j) = SolVec(1);
        theta1(j) = SolVec(2);
        theta2(j) = SolVec(3);
        
        % The initial guess fo the next time-step's angle solutions must be
        % the solution for the current time-step
        InitVarVec = [
            gamma(j);
            theta1(j);
            theta2(j)];
    end
    
    % With the link angles know across the duration of the run, the link
    % angular velocites may be found via numerical differentiation
    gamma_dot = gradient(gamma,dt);
    theta1_dot = gradient(theta1,dt);
    theta2_dot = gradient(theta2,dt);
    
    % Also find link angular accelerations via numerical differentiation
    gamma_dot_dot = gradient(gamma_dot,dt);
    theta1_dot_dot = gradient(theta1_dot,dt);
    theta2_dot_dot = gradient(theta2_dot,dt);
    
    % With all link kinematics known, the motor torques necessary for
    % generating the desired motion may be found.
    % Note: The following expressions have been derived using a kinetics
    % analysis of the robot. See the code titled "Syms_kinematics.m" for
    % the symbolic processing algorithm that was used to generate the
    % following equations.
    T_gamma =...
        c(1)*gamma_dot - Robot.m*(Robot.L2*cosd(theta1 + theta2)...
        + Robot.L1*cosd(theta1)).*(2*Robot.L2*sind(theta1 + theta2).*...
        gamma_dot.*theta1_dot - Robot.L1*cosd(theta1).*gamma_dot_dot...
        - Robot.L2*cosd(theta1 + theta2).*gamma_dot_dot +...
        2*Robot.L2*sind(theta1 + theta2).*gamma_dot.*theta2_dot...
        + 2*Robot.L1*sind(theta1).*gamma_dot.*theta1_dot);
    T_theta1 =...
        c(2)*theta1_dot + Robot.m*(Robot.L1^2*theta1_dot_dot...
        + Robot.L2^2*theta1_dot_dot + Robot.L2^2*theta2_dot_dot +...
        (Robot.L1^2*sind(2*theta1).*gamma_dot.^2)/2 -...
        Robot.L1*Global.g*cosd(theta1) -...
        Robot.L2*Global.g*cosd(theta1 + theta2) +...
        (Robot.L2^2*sind(2*theta1 + 2*theta2).*gamma_dot.^2)/2 +...
        2*Robot.L1*Robot.L2*cosd(theta2).*theta1_dot_dot +...
        Robot.L1*Robot.L2*cosd(theta2).*theta2_dot_dot +...
        Robot.L1*Robot.L2*sind(2*theta1 + theta2).*gamma_dot.^2 -...
        Robot.L1*Robot.L2*sind(theta2).*theta2_dot.^2 -...
        2*Robot.L1*Robot.L2*sind(theta2).*theta1_dot.*theta2_dot);
    T_theta2 =...
        c(3)*theta2_dot + Robot.L2*Robot.m*(Robot.L2*theta1_dot_dot -...
        Global.g*cosd(theta1 + theta2) + Robot.L2*theta2_dot_dot +...
        (Robot.L1*sind(theta2).*gamma_dot.^2)/2 +...
        (Robot.L2*sind(2*theta1 + 2*theta2).*gamma_dot.^2)/2 +...
        Robot.L1*sind(theta2).*theta1_dot.^2 +...
        Robot.L1*cosd(theta2).*theta1_dot_dot +...
        (Robot.L1*sind(2*theta1 + theta2).*gamma_dot.^2)/2);
    
    % Save all link motion and motor torque data to the appropriate arrays
    % for neural network training and validation
    GlobalInputData(:,BaseNum + 1:BaseNum + N) = [
        AngleLim(gamma);
        AngleLim(theta1);
        AngleLim(theta2);
        gamma_dot;
        theta1_dot;
        theta2_dot;
        gamma_dot_dot;
        theta1_dot_dot;
        theta2_dot_dot];
    GlobalOutputData(:,BaseNum + 1:BaseNum + N) = [
        T_gamma;
        T_theta1;
        T_theta2];
    BaseNum = BaseNum + N;
end