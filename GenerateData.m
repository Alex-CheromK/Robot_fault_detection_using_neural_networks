function [GlobalInputData,GlobalOutputData,FaultVec] = GenerateData(NumRuns,FaultProb,Global,Work,Robot)
N = 20;
TimeVec = linspace(0,5,N);
dt = TimeVec(2) - TimeVec(1);

gamma = zeros(1,N);
theta1 = zeros(1,N);
theta2 = zeros(1,N);

% Start point
rp1 = [Work.dx 0]';

%
InitVarVec = [0 -45 90]';
GlobalInputData = zeros(9,N*NumRuns);
GlobalOutputData = zeros(3,N*NumRuns);
FaultVec = zeros(3,NumRuns);
BaseNum = 0;
for i = 1:NumRuns
    % Fault settings
    if FaultProb > 0
        c = [10 10 10];
        if rand <= FaultProb
            c(1) = 20;
            FaultVec(1,i) = 1;
        end
        if rand <= FaultProb
            c(2) = 20;
            FaultVec(2,i) = 1;
        end
        if rand <= FaultProb
            c(3) = 20;
            FaultVec(3,i) = 1;
        end
    else
        c = [10 10 10];
    end
    
    % Select next search point
    rp2 = [
        sign(-1 + rand*2)*(Work.dx + rand*Work.W);
        -Work.L/2 + rand*Work.L];
    
    % Define ellispe in local frame
    a = norm(rp2 - rp1)/2;
    b = 0.5;
    xe = linspace(0,2*a,N);
    ze = b*sqrt(1 - ((xe - a)/a).^2);
    
    % Convert ellipse to global frame
    u = (rp2 - rp1)/norm(rp2 - rp1);
    rm = [
        rp1 + xe.*u;
        ze];
    
    %
    rp1 = rp2;
    
    %
    for j = 1:N
        SolVec = fsolve(@(VarVec) RobotKinematics(VarVec,rm(:,j),Robot),InitVarVec);
        gamma(j) = SolVec(1);
        theta1(j) = SolVec(2);
        theta2(j) = SolVec(3);
        
        %
        InitVarVec = [
            gamma(j);
            theta1(j);
            theta2(j)];
    end
    
    %
    gamma_dot = gradient(gamma,dt);
    theta1_dot = gradient(theta1,dt);
    theta2_dot = gradient(theta2,dt);
    
    %
    gamma_dot_dot = gradient(gamma_dot,dt);
    theta1_dot_dot = gradient(theta1_dot,dt);
    theta2_dot_dot = gradient(theta2_dot,dt);
    
    %
    T_gamma = c(1)*gamma_dot - Robot.m*(Robot.L2*cosd(theta1 + theta2) + Robot.L1*cosd(theta1)).*...
        (2*Robot.L2*sind(theta1 + theta2).*gamma_dot.*theta1_dot -...
        Robot.L1*cosd(theta1).*gamma_dot_dot - Robot.L2*cosd(theta1 + theta2).*gamma_dot_dot +...
        2*Robot.L2*sind(theta1 + theta2).*gamma_dot.*theta2_dot + 2*Robot.L1*sind(theta1).*gamma_dot.*theta1_dot);
    T_theta1 = c(2)*theta1_dot + Robot.m*(Robot.L1^2*theta1_dot_dot + Robot.L2^2*theta1_dot_dot + Robot.L2^2*theta2_dot_dot +...
        (Robot.L1^2*sind(2*theta1).*gamma_dot.^2)/2 - Robot.L1*Global.g*cosd(theta1) - Robot.L2*Global.g*cosd(theta1 + theta2) +...
        (Robot.L2^2*sind(2*theta1 + 2*theta2).*gamma_dot.^2)/2 + 2*Robot.L1*Robot.L2*cosd(theta2).*theta1_dot_dot +...
        Robot.L1*Robot.L2*cosd(theta2).*theta2_dot_dot + Robot.L1*Robot.L2*sind(2*theta1 + theta2).*gamma_dot.^2 -...
        Robot.L1*Robot.L2*sind(theta2).*theta2_dot.^2 - 2*Robot.L1*Robot.L2*sind(theta2).*theta1_dot.*theta2_dot);
    T_theta2 = c(3)*theta2_dot + Robot.L2*Robot.m*(Robot.L2*theta1_dot_dot - Global.g*cosd(theta1 + theta2) + Robot.L2*theta2_dot_dot +...
        (Robot.L1*sind(theta2).*gamma_dot.^2)/2 + (Robot.L2*sind(2*theta1 + 2*theta2).*gamma_dot.^2)/2 +...
        Robot.L1*sind(theta2).*theta1_dot.^2 + Robot.L1*cosd(theta2).*theta1_dot_dot + (Robot.L1*sind(2*theta1 + theta2).*gamma_dot.^2)/2);
    
    % Save data to global collection
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