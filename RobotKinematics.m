% Current function outputs the error between actual package position and
% the package position that is being guess by the nonlinear equations
% solver.
% Through this error value, the equation solver can iteratively converge
% toward the correct solution.
% This function takes in the following arguments:
% 1. VarVec - A vector containing the equation solver's current best guess
% for the solution.
% 2. r_m - The current location of the package in 3D space.
% 3. Robot - A class object containing robot properties.

function Output = RobotKinematics(VarVec,r_m,Robot)
% Extract link angles from the variable vector
gamma = VarVec(1);
theta1 = VarVec(2);
theta2 = VarVec(3);

% Compute 3D position of the package corresponding the current guessed
% angles
TmpVec = [
    cosd(gamma)*cosd(theta1)*(Robot.L1 + Robot.L2*cosd(theta2)) - Robot.L2*cosd(gamma)*sind(theta1)*sind(theta2);
    cosd(theta1)*sind(gamma)*(Robot.L1 + Robot.L2*cosd(theta2)) - Robot.L2*sind(gamma)*sind(theta1)*sind(theta2);
    -sind(theta1)*(Robot.L1 + Robot.L2*cosd(theta2)) - Robot.L2*cosd(theta1)*sind(theta2)];

% Compute the error between the actual and guessed position
Output = TmpVec - r_m;