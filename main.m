clear
clc

global k1 k2 l b I m g
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


init_state = [0 0 3 0 0 0 0 0 0 0 0 0];
sim('main_simulink');

temp = 1;
