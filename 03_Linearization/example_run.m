% example_run.m
% Demonstrates the full linearization post-processing workflow:
%   1. Load and adjust the SHARPy linear state-space system
%   2. Save it as a .mat file for use in MATLAB
%   3. Run an open-loop gust response simulation

clear; clc;

%% Paths
route_directory = fileparts(mfilename('fullpath'));
case_name = 'superflexop_cruise_linear_FOM_free_flight_IPdamped_num_modes21';
SHARPy_case_output_folder = fullfile(route_directory, 'output');

%% Postprocess SHARPy linear system and save as .mat
postprocess_linear_system(route_directory, case_name, SHARPy_case_output_folder);

%% Load saved state-space system
load(fullfile(route_directory, 'linear_statespace_files_matlab', strcat(case_name, '.mat')));

%% Compute open-loop gust response
gust_intensities = [0.1, 0.01, 0.001];
gust_lengths = [10];
output_folder = fullfile(route_directory, '..', 'output', case_name);
get_linear_gust_response(state_space_system, input_settings, gust_lengths, gust_intensities, output_folder);