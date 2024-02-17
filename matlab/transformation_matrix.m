syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 real
syms a1 a2 a3 a4 a5 a6 real
syms d1 d2 d3 d4 d5 d6 real
syms theta1 theta2 theta3 theta4 theta5 theta6 real

% Define DH parameters as symbolic variables
% alpha a d theta
DH = [deg2rad(alpha1)     a1     d1        deg2rad(theta1);
      deg2rad(alpha2)     a2     d2        deg2rad(theta2);
      deg2rad(alpha3)     a3     d3        deg2rad(theta3);
      deg2rad(alpha4)     a4     d4        deg2rad(theta4);
      deg2rad(alpha5)     a5     d5        deg2rad(theta5);
      deg2rad(alpha6)     a6     d6        deg2rad(theta6)];

% Call the robotTransform function

T = robotTransform(DH);


% Display the resulting transformation matrix
%disp('Transformation Matrix:');
disp(T);


