clear all

route_directory = '/home/sduess/Documents/SHARPY_simulations/GLA_systems_for_very_flexible_aircraft/03_Linearization';
case_name ='superflexop_cruise_linear_FOM_clamped'; %'superflexop_cruise_linear_FOM_free_flight';
SHARPy_case_output_folder = '/home/sduess/Documents/SHARPY_simulations/GLA_systems_for_very_flexible_aircraft/lib/sharpy/output/';

postprocess_linear_system(route_directory, case_name, SHARPy_case_output_folder);

load(strcat('./linear_statespace_files_matlab/', case_name, '.mat'));
gust_intensities = [0.1, 0.01, 0.001];
gust_lengths = [10];
get_linear_gust_response(state_space_system, input_settings, gust_lengths, gust_intensities);