function [sys_kf,sys_LQR, sys_final, controller_final] = controller_synthesis(input_settings,state_space_system_parameter)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    A = state_space_system_parameter.A;
    B = [state_space_system_parameter.B_cs state_space_system_parameter.B_gust];
    C = state_space_system_parameter.C;
    D = [state_space_system_parameter.D_cs state_space_system_parameter.D_gust];


    if input_settings.get_thrust
        B = [B state_space_system_parameter.B_thrust];
        D = [D state_space_system_parameter.B_thrust];
        num_thrust_inputs = size(state_space_system_parameter.B_thrust, 2);
    else
        num_thrust_inputs = 0;
    end
    C = C(input_settings.sensors,:);
    D = D(input_settings.sensors,:);
       
    sys = ss(A,B,C,D, input_settings.dt); % Plant dynamics and additive input noise w

   
%% Remove aileron inputs (except for outboard one)
if ~input_settings.rbm
    sys_reduced = remove_inputs_from_state_space_model(sys, [5,6, 11,12], true);
else
    % TODO include elevator in GLA controller
    sys_reduced = remove_inputs_from_state_space_model(sys, [5,6, 11,12], true);
end
%% reduce cs inputs by assuming symmetry
num_ailerons = 8;
num_elevators = 0;
% if input_settings.rbm
%     num_elevators = 4;
% end
num_control_surfaces = num_ailerons + num_elevators;
num_aero_states = input_settings.num_aero_states;
num_modes = input_settings.num_modes;
num_ignored_last_columns = 1 + num_thrust_inputs; % Gust (1) + thrust inputs
num_rbm = int8(input_settings.rbm)*9;
sys_symmetric_inputs = make_control_inputs_symmetric(sys_reduced, num_control_surfaces/2, num_ignored_last_columns);

%% Reduce cs inputs by assuming same deflection/join ailerons to one
num_cs = num_control_surfaces / 2;
if input_settings.rbm
    % TODO: join ailerons and elevators separataley for free-flying
    % sys_final = sys_symmetric_inputs;

    sys_final = join_control_surfaces(sys_symmetric_inputs, num_cs, num_ignored_last_columns);
    num_cs = 1;
else
    sys_final = join_control_surfaces(sys_symmetric_inputs, num_cs, num_ignored_last_columns);
    num_cs = 1;
end

%% Design LQR controller

sys_LQR = ss(sys_final.A, sys_final.B(:,1:end-(num_thrust_inputs)), sys_final.C, sys_final.D(:,1:end-(num_thrust_inputs)), input_settings.dt);
if input_settings.rbm
    penalized_modes = [1];
else
    % Symmetrical modes
    penalized_modes = [1, 2];
end
% controller_final = 1;
controller_final = setup_controller_struct(sys_LQR, size(sys_LQR.B,2)-1, ...
    num_aero_states, num_modes,num_rbm, penalized_modes, ...
    5000., ...
    1);

%% Design LQG controller
% TODO: Thrust
A_kf = sys_LQR.A(1:end-num_cs,1:end-num_cs);
B_kf = [sys_LQR.B(1:end-num_cs, 1:num_cs) sys_LQR.A(1:end-num_cs,end-num_cs+1:end) sys_LQR.B(1:end-num_cs,end)];
C_kf = sys_LQR.C(:,1:end-num_cs);
D_kf =  [sys_LQR.D(:, 1:num_cs) sys_LQR.C(:, end-num_cs+1:end) sys_LQR.D(:,end)];

sys_kf =  ss(A_kf, B_kf, C_kf, D_kf, input_settings.dt);
end

