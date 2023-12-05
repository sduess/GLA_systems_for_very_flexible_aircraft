function setup_LQG_controller(route_directory, case_name, use_elevator, ...
    LQR_tuning, design_name, accelerator_sensors, sensors_only_z,only_pos,rotation_dot,...
    join_cs, symmetric_cs, modes_to_be_removed)

addpath(strcat(route_directory,'/../05_Utils/matlab_functions/'));
%% Load state space information and input_settings
load(strcat(route_directory, '/../03_Linearization/linear_statespace_files_matlab/', case_name, '.mat'));

%% Get Sensor measurements
input_settings.index.eta_dot_tip = input_settings.index.eta_tip +input_settings.n_nodes * 6;
if sensors_only_z
    eta_tip = [input_settings.index.eta_tip([3, 9])]; 
    eta_dot_tip = [input_settings.index.eta_dot_tip([3, 9])]; 
    if ~only_pos
        eta_tip = [eta_tip input_settings.index.eta_tip([5, 11])];
        if rotation_dot
            eta_dot_tip = [eta_dot_tip input_settings.index.eta_dot_tip([5, 11])];
        end
    end
else
    eta_tip = input_settings.index.eta_tip;
    eta_dot_tip = input_settings.index.eta_dot_tip;
end

if accelerator_sensors
     input_settings.sensors = eta_tip + input_settings.n_nodes * 2 * 6; 
else
    input_settings.sensors = [eta_tip eta_dot_tip]; 
end
if input_settings.rbm
    input_settings.sensors = [input_settings.sensors input_settings.index.rbm]; 
end

%% Get LQR Controller
% Remove unused 
[sys_kf, sys_LQR,sys_final, controller_final, input_settings] = controller_synthesis(input_settings, ...
    state_space_system, use_elevator, LQR_tuning, join_cs, symmetric_cs, ...
    modes_to_be_removed);

%% Save controller parameters
if use_elevator
    case_name= strcat(case_name,  '_elev_on');
end
case_name = strcat(case_name, design_name);
save(strcat(route_directory,'/controller_files_matlab/',case_name), ...
    'sys_kf', ...
    'sys_LQR', ...
    'sys_final', ...
    'controller_final', ...
    'input_settings', ...
    'case_name', ...
    '-v7.3');
end
