function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;


% FILL IN YOUR CODE HERE

k_p=;
k_v=;

e=s_des-s;
z_des_2ord=u/params.mass - params.gravity;

u=params.mass*(z_des_2ord + k_v*e(1) + k_p*e(2) + params.gravity);



end

