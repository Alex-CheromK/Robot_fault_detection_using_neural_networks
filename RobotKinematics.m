function Output = RobotKinematics(VarVec,r_m,Robot)

gamma = VarVec(1);
theta1 = VarVec(2);
theta2 = VarVec(3);

TmpVec = [
    cosd(gamma)*cosd(theta1)*(Robot.L1 + Robot.L2*cosd(theta2)) - Robot.L2*cosd(gamma)*sind(theta1)*sind(theta2);
    cosd(theta1)*sind(gamma)*(Robot.L1 + Robot.L2*cosd(theta2)) - Robot.L2*sind(gamma)*sind(theta1)*sind(theta2);
    -sind(theta1)*(Robot.L1 + Robot.L2*cosd(theta2)) - Robot.L2*cosd(theta1)*sind(theta2)];

Output = TmpVec - r_m;