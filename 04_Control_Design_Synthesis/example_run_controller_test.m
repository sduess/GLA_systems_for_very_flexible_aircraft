clear all
%% Define directories and case name

route_directory = '/home/sduess/Documents/SHARPY_simulations/GLA_systems_for_very_flexible_aircraft/04_Control_Design_Synthesis/';
case_name ='superflexop_cruise_linear_ROM_free_flight';
simulink_file_directory = '/home/sduess/Documents/SHARPY_simulations/GLA_systems_for_very_flexible_aircraft/05_Utils/simulink_files/'%strcat(route_directory, '/../05_Utils/simulink_files/');
design_name =  '_acc1_onlyz1_onlypos1'
case_name = strcat(case_name,design_name) ; 

%% Load Data
load(strcat('./controller_files_matlab/', case_name, '.mat'));
%% Define Simulation, Controller, and Gust Parameter
input_settings.flight_time= 3;
gust_lengths = [5, 10, 20, 40, 80];
gust_intensities = [0.1];
controller_inputs = struct();

%% Run open-loop simulation with ROM in the loop
controller_inputs.controller_name = 'Open-Loop';
run_closed_loop_discrete_gust_simulations(gust_lengths, ...
    gust_intensities, input_settings, controller_inputs, case_name, ...
    route_directory,simulink_file_directory, ...
    sys_final, sys_kf, 0)
%% Run closed loop LQR simulation with ROM in the loop
controller_inputs.controller_name = 'LQR';
run_closed_loop_discrete_gust_simulations(gust_lengths, ...
    gust_intensities, input_settings, controller_inputs, case_name, ...
    route_directory,simulink_file_directory, ...
    sys_final, sys_kf, controller_final.K)

%% Run closed loop LQG simulation with ROM in the loop
controller_inputs.controller_name = 'LQG';
controller_inputs.Qn = 2.3;
controller_inputs.Rn=1e-6; 

run_closed_loop_discrete_gust_simulations(gust_lengths, ...
    gust_intensities, input_settings, controller_inputs, case_name, ...
    route_directory,simulink_file_directory, ...
    sys_final, sys_kf, controller_final.K)

%% TODO: Comment on linear FOM in the loop maybe add simulations to see difference for high gust lengths
