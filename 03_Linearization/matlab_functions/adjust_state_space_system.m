function state_space_parameter = adjust_state_space_system(state_space_system, input_settings)
%convert_state_space_for_LQR Adjust extracted state space system
%   The loaded state space system exported from SHARPy is adjusted here for
%   the closed-loop simulations. This includes:
%       - removing all unused inputs and rearranging for example the
%         deflection of a control surface and its rate as both depend on
%         each other
%       - extracts the column from the B and D matrices that describe the
%         effect of the gust and thrust input to the states if specified


%% Drop integrals of RBM necessary to design LQR
orig_size_A = size(state_space_system.A,1);
if input_settings.rbm && input_settings.rom_order > 0
    drop_begin = orig_size_A- 2 * input_settings.num_modes + 1; 
    drop_end = drop_begin+8;

    state_space_system.A(:,drop_begin:drop_end) =  [];
    state_space_system.A(drop_begin:drop_end,:) =  [];
    state_space_system.B(drop_begin:drop_end,:) = [];
    state_space_system.C(:,drop_begin:drop_end) =  [];
end


%% Reduce model with not used inputs
idx_input_end =input_settings.control_input_start + 2  * input_settings.num_control_surfaces - 1;
idx_inputs = [input_settings.control_input_start:idx_input_end];
state_space_parameter.B_cs = state_space_system.B(:,idx_inputs);
state_space_parameter.D_cs = state_space_system.D(:,idx_inputs);  


%% Extract delta to state
new_A_colum = state_space_parameter.B_cs(:,1 :input_settings.num_control_surfaces);
A = [state_space_system.A new_A_colum; ...
    zeros(input_settings.num_control_surfaces,...
         (size(state_space_system.A,2)+ input_settings.num_control_surfaces))];

for counter = 0:1:input_settings.num_control_surfaces-1
    A(size(A,1)-counter,size(A,2)-counter) = 1; % Linear integration of delta dot
end
state_space_parameter.A = A;

%Delete delta input and add delta_dot influence on delta 
state_space_parameter.B_cs(:,1:input_settings.num_control_surfaces) = [];
state_space_parameter.state_space_parameter.B_cs = [state_space_parameter.B_cs; eye(input_settings.num_control_surfaces) * state_space_system.Ts]; 

% Adding feed through of the delta-input on the output on C as a new column
% and deleting column added in C out of D; 
state_space_parameter.C = [state_space_system.C state_space_parameter.D_cs(:,1:input_settings.num_control_surfaces)];
state_space_parameter.D_cs(:, 1:input_settings.num_control_surfaces) = []; 


%% Get Gust Disturbance Matrices
if input_settings.get_gust
    state_space_parameter.B_gust = [state_space_system.B(:,input_settings.gust_input); zeros(input_settings.num_control_surfaces, 1)];
    state_space_parameter.D_gust = state_space_system.D(:,input_settings.gust_input);
end

%% Get Thrust if applicable
if input_settings.get_thrust
    state_space_parameter.B_thrust = [state_space_system.B(:,input_settings.thrust_input); zeros(input_settings.num_control_surfaces, 1)];
    state_space_parameter.D_thrust = state_space_system.D(:,input_settings.thrust_input);
end

end