function [ omega ] = controller_angular_velocity( u )
%CONTROLLER_ANGULAR_VELOCITY Summary of this function goes here
%   Detailed explanation goes here
global omega2_last_step K I
desired_angular_accel = [u(1) u(2) u(3)]';
actual_angular_accel = [u(4) u(5) u(6)]';

desired_delta_omega2 = inv(K)*I*(desired_angular_accel - actual_angular_accel);
desired_omega2 = omega2_last_step + desired_delta_omega2;
omega2_last_step = desired_omega2;
omega = sqrt(desired_omega2);


end

