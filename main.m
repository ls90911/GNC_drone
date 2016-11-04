clear
clc

global k1 k2 l b I m g omega2_last_step K
%% drone parameters
k1 = 6.11*10^-8;
k2 = 1.5*10^-9;
l = 0.175;
b = 0.175;
I_xx = 2.32*10^-3;
I_yy = 2.32*10^-3;
I_zz = 4.00*10^-3;
m = 0.5;
g = 9.8;
I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];
K = [-k1*b k1*b k1*b -k1*b;k1*l k1*l -k1*l -k1*l;k2 -k2 k2 -k2];

%% controller parameter
k_p_p = 1;
k_i_p = 0;
k_d_p = 0;

k_p_q = 1;
k_i_q = 0;
k_d_q = 0;

k_p_r = 1;
k_i_r = 0;
k_d_r = 0;
%% 
omega2_last_step = [0 0 0 0]';

init_state = [0 0 3 0 0 0 0 0 0 0 0 0];
sim('main_simulink');

temp = 1;
