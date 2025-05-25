%% Example run to get an LQG controller
% Just an example how to get the LQG controller from the linear model
% 'Post-phd steffi' didn't have time yet to clean the work from 'very-close to
% phd deadline steffi' :) Sorry if the code is not that clean.

%% Define directories and parameters
% TODO: get rid of absolute path
route_directory = 'C:\Users\User\Documents\GLA_systems_for_very_flexible_aircraft\04_Control_Design_Synthesis/';
case_name ='superflexop_cruise_linear_FOM_clamped_IPdamped_wing_only_num_modes12';

%% Set penalties for LQR tuning
input_LQR_tuning = struct();
input_LQR_tuning.idx_penalized_aero_state = []; 
input_LQR_tuning.weights_for_penalized_aero_state =  []; 
input_LQR_tuning.idx_penalized_modal_displacement =[1, 2]; 
input_LQR_tuning.weights_for_penalized_modal_displacement = [140, 140];
input_LQR_tuning.idx_penalized_modal_velocities =1:2; %
input_LQR_tuning.weights_for_penalized_modal_velocities =[0.0, 0.0]
input_LQR_tuning.idx_penalized_rbm_modes = []; 
input_LQR_tuning.weights_for_penalized_rbm_modes = []; 
input_LQR_tuning.initial_diagonal_values = 0; 
input_LQR_tuning.weights_for_penalized_control_surfaces = 2e-4;
input_LQR_tuning.R_values = 1; 

design_name = '';
%% Create systems and control gains for LQG controller
use_elevators=false;
accelerator_sensors=false;
sensors_only_z = true;
only_pos = false;
rotation_dot = false;
join_cs = true;
make_cs_symmetric = false;
modes_to_be_removed = []; 
design_name = strcat(design_name, '_acc', num2str(accelerator_sensors), ...
    '_onlyz', num2str(sensors_only_z),...
    '_onlypos', num2str(only_pos),...
    '_rotdot', num2str(rotation_dot), ...
    '_joinedcs', num2str(join_cs),...
    '_make_cs_symmetric', num2str(make_cs_symmetric), ...
    '_woIP_wo5thBM');

setup_LQG_controller(route_directory, ...
                     case_name, ...
                     use_elevators, ...
                     input_LQR_tuning, ...
                     design_name, ...
                     accelerator_sensors, ...
                     sensors_only_z, ...
                     only_pos, ...
                     rotation_dot, ...
                     join_cs, ...
                     make_cs_symmetric, ...
                     modes_to_be_removed);

%% Test LQR controller on a discrete gust with H=10m and I=10%
if use_elevators
    case_name = strcat(case_name, '_elev_on');
end
case_name = strcat(case_name, design_name);
load(strcat('./controller_files_matlab/', case_name, '.mat'));
% Define directory where to find simulink files
addpath(strcat(route_directory,'/../05_Utils/matlab_functions/'));
simulink_file_directory = strcat(route_directory, '/../05_Utils/simulink_files/');

% Get gust input
gust_time_series = get_1minuscosine_gust_input(10, ...
                                                0.1, ...
                                                input_settings.dt, ...
                                                input_settings.u_inf, ...
                                                input_settings.flight_time);

% Open-loop gust respons
simulation_output_open_loop = run_linear_closed_loop_LQR_response(sys_final, ...
                    input_settings,...
                    gust_time_series, ...
                    controller_final.K.*0, ...
                    simulink_file_directory);

simulation_output = run_linear_closed_loop_LQR_response(sys_final, ...
                    input_settings,...
                    gust_time_series, ...
                    controller_final.K, ...
                    simulink_file_directory);

%% Plot
num_cs = size(sys_LQR.B,2)-1
figure(1); hold on;plot(simulation_output.actual_output.Time, simulation_output.actual_output.Data(:,input_settings.index.eta_tip([3 3+6]))./(7.07/2)*100+6.2); 
plot(simulation_output_open_loop.actual_output.Time, simulation_output_open_loop.actual_output.Data(:,input_settings.index.eta_tip([3 3+6]))./(7.07/2)*100+6.2, 'k-'); hold off;
 figure(2); hold on;plot(simulation_output.control_input.Time, rad2deg(simulation_output.control_input.Data(:,1:num_cs)));hold off;
 figure(3); hold on;plot(simulation_output.control_input.Time, rad2deg(simulation_output.control_input.Data(:,num_cs+1:2*num_cs)));hold off;
.Time, rad2deg(simulation_output.control_input.Data(:,1:num_cs/2)));hold off;
 figure(5); hold on;plot(simulation_output.control_input.Time, rad2deg(simulation_output.control_input.Data(:,num_cs+1:num_cs + num_cs/2)));hold off;
figure(6); hold on;plot(simulation_output.actual_output.Time, simulation_output.actual_output.Data(:,input_settings.index.eta_tip([3 3+6]))./(7.07/2)*100+6.2); 


%% Get Poles
num_cs = size(sys_LQR.B,2)-1
system_open_loop = ss(sys_LQR.A(1:end-num_cs:1:end-num_cs,1:end-num_cs), sys_LQR.B(1:end-num_cs,:), sys_LQR.C(:,1:end-num_cs), sys_LQR.D, input_settings.dt);
system_closed_loop = ss(sys_LQR.A-sys_LQR.B(:,1:num_cs) * controller_final.K, ...
        sys_LQR.B(:,end),...
        sys_LQR.C-sys_LQR.D(:,1:num_cs)*controller_final.K,...
        sys_LQR.D(:,end), ...
        input_settings.dt)

fig = figure(10)
hold on;

pzmap(system_open_loop, 'k')
pzmap(system_closed_loop, 'r') 
zgrid()

