function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
k_p_y = 30;
k_v_y = 10;
k_p_z = 500;
k_v_z = 10;
k_p_phi = 1000;
k_v_phi = 100;

phi_dot_dot_c = 0;
phi_dot_c = 0;

mass = params.mass;
gravity = params.gravity;
I_xx = params.Ixx;

y = state.pos(1);
z = state.pos(2);
y_dot = state.vel(1);
z_dot = state.vel(2);
y_0 = des_state.pos(1);
z_0 = des_state.pos(2);
y_dot_dot_T = des_state.acc(1);
y_dot_T = des_state.vel(1);
phi_dot = state.omega(1);
phi = state.rot(1);

%fprintf('y des %f\t', y_0);
%fprintf('z des %f\n', z_0);

phi_c = -1 / gravity * (y_dot_dot_T + k_v_y * (y_dot_T - y_dot) + k_p_y * (y_0 - y));
u1 = mass * (gravity - k_v_z * z_dot + k_p_z * (z_0 - z));
u2 = I_xx * (phi_dot_dot_c + k_v_phi * (phi_dot_c - phi_dot) + k_p_phi * (phi_c - phi));

end

