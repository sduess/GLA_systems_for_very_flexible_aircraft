function setup_LQG_controller(route_directory, case_name, use_elevator)

addpath(strcat(route_directory,'/../05_Utils/matlab_functions/'));
%% Load state space information and input_settings
load(strcat(route_directory, '/../03_Linearization/linear_statespace_files_matlab/', case_name, '.mat'));

%% Get Sensor measurements
    input_settings.sensors = [input_settings.index.eta_tip input_settings.index.eta_dot_tip]; %tip deflection right[1816 1817 1818 2248 2249 2250 4159 4161 4163 4166]; %1:size(sys_final.C,1); %sensor_measurements
    if input_settings.rbm
        input_settings.sensors = [input_settings.sensors input_settings.index.rbm]; 
    end

%% Get LQR Controller
% Remove unused 
[sys_kf, sys_LQR,sys_final, controller_final] = controller_synthesis(input_settings,state_space_system, use_elevator);

%% Save controller parameters
if use_elevator
    case_name= strcat(case_name,  '_elev_on');
end
save(strcat(route_directory,'/controller_files_matlab/',case_name), ...
    'sys_kf', ...
    'sys_LQR', ...
    'sys_final', ...
    'controller_final', ...
    'input_settings', ...
    'case_name', ...
    '-v7.3');
end
