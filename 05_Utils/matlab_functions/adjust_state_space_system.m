function adjusted_sys = adjust_state_space_system(state_space_system, input_settings)
%adjust_state_space_system Adjust SHARPy state space system for simulation
%   Processes the state space system exported from SHARPy and returns an
%   adjusted system struct (adjusted_sys) ready for closed-loop simulation.
%   Adjustments include:
%       - dropping RBM integral states (if ROM and RBM are active)
%       - extracting control surface input columns (B_cs, D_cs) and
%         integrating control surface deflection (delta) as an augmented
%         state, so the input becomes delta_dot
%       - feeding delta through to the output matrix C
%       - extracting gust disturbance columns (B_gust, D_gust) if requested
%       - extracting thrust input columns (B_thrust, D_thrust) if requested
%
%   Inputs:
%       state_space_system  - struct with fields A, B, C, D, Ts as exported
%                             by SHARPy (read via read_SHARPy_state_space_system)
%       input_settings      - struct with case parameters (see set_input_parameters
%                             and the corresponding JSON parameter file)
%
%   Output:
%       adjusted_sys        - struct containing the adjusted system matrices:
%                             A, B_cs, C, D_cs, and optionally B_gust,
%                             D_gust, B_thrust, D_thrust


%% Drop integrals of RBM necessary to design LQR
orig_size_A = size(state_space_system.A,1);
if input_settings.rbm && input_settings.rom_order > 0
    drop_begin = orig_size_A- 2 * input_settings.num_modes + 1;
    list_idx_state_removed = drop_begin:drop_begin + 9 - 1; % 9 rbm modes
    state_space_system = remove_state_from_state_space_model(state_space_system, list_idx_state_removed);
end


%% Extract control surface input columns
idx_input_end =input_settings.control_input_start + 2  * input_settings.num_control_surfaces - 1;
idx_inputs = input_settings.control_input_start:idx_input_end;
adjusted_sys.B_cs = state_space_system.B(:,idx_inputs);
adjusted_sys.D_cs = state_space_system.D(:,idx_inputs);


%% Extract delta to state
new_A_column = adjusted_sys.B_cs(:,1:input_settings.num_control_surfaces);
A = [state_space_system.A new_A_column; ...
    zeros(input_settings.num_control_surfaces,...
         (size(state_space_system.A,2)+ input_settings.num_control_surfaces))];

for counter = 0:input_settings.num_control_surfaces-1
    A(size(A,1)-counter,size(A,2)-counter) = 1; % Linear integration of delta dot
end
adjusted_sys.A = A;

%Delete delta input and add delta_dot influence on delta
adjusted_sys.B_cs(:,1:input_settings.num_control_surfaces) = [];
adjusted_sys.B_cs = [adjusted_sys.B_cs; eye(input_settings.num_control_surfaces) * state_space_system.Ts];

% Adding feed through of the delta-input on the output on C as a new column
% and deleting column added in C out of D;
adjusted_sys.C = [state_space_system.C adjusted_sys.D_cs(:,1:input_settings.num_control_surfaces)];
adjusted_sys.D_cs(:, 1:input_settings.num_control_surfaces) = [];


%% Get Gust Disturbance Matrices
if input_settings.get_gust
    adjusted_sys.B_gust = [state_space_system.B(:,input_settings.gust_input); zeros(input_settings.num_control_surfaces, 1)];
    adjusted_sys.D_gust = state_space_system.D(:,input_settings.gust_input);
end

%% Get Thrust if applicable
if input_settings.get_thrust
    adjusted_sys.B_thrust = [state_space_system.B(:,input_settings.thrust_input); zeros(input_settings.num_control_surfaces, 1)];
    adjusted_sys.D_thrust = state_space_system.D(:,input_settings.thrust_input);
end

end