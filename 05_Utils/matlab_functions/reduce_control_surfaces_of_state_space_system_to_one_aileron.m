function sys_final = reduce_control_surfaces_of_state_space_system_to_one_aileron(state_space_system_parameter,input_settings)
%UNTITLED3 Summary of this function goes here
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

end