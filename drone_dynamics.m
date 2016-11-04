function [ d_state ] = drone_dynamics( u )
%DRONE_DYNAMICS Summary of this function goes here
%   Detailed explanation goes here
global k1 k2 l b I m g K
% x_earth = u(1);
% y_earth = u(2);
% z_earth = u(3);
v_x_body = u(4);
v_y_body = u(5);
v_z_body = u(6);
phi = u(7);
theta = u(8);
psi = u(9);
p = u(10);
q = u(11);
r = u(12);
omega2 = [u(13)^2 u(14)^2 u(15)^2 u(16)^2]';
%omega = [u(13) u(14) u(15) u(16)]';
F = k1*sum(omega2);
%X = [x_earth y_earth z_earth]';
V = [v_x_body v_y_body v_z_body]';
%Eta = [phi theta psi]';
Omega = [p q r]';



R_d_angle = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

% inv_R_d_angle = [1 0 -sin(theta);...
%      0 cos(phi) cos(theta)*sin(phi);...
%      0 -sin(theta) cos(theta)*cos(phi)];
 
 R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(phi) -sin(theta);...
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
 
 R_B_E = [cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);...
      sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);...
       -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
   
dX = R_B_E*V;
dV = R_E_B*[0 0 -g]' +[0 0 -F]'/m -cross(Omega,V);
dEta = R_d_angle * Omega;
dOmega = inv(I)*(cross(-Omega,I*Omega)+K*omega2);

d_state = [dX;dV;dEta;dOmega];
end

