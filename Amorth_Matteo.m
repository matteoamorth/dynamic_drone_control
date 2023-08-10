clear
close all
clc

%% Variables

gravity = 9.81;
drone_mass = 0.5;
drone_inertia = 0.05;

th_force = sdpvar(1,1);
control_torque = sdpvar(1,1);
velocity = sdpvar(1,1);
angular_v = sdpvar(1,1);

f_g_less = th_force - drone_mass * gravity;