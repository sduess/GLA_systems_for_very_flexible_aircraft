import json, os
import numpy as np


file_dir = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))

case_name ='superflexop_cruise_linear_FOM_free_flight'
state_space_information = {
    'aircraft_model': 'flexop',
    'case_name':case_name,
'u_inf': 45,
'rom_order': 0,
'num_modes': 30,
'num_aero_states': 21673,
'rbm': True, 
'flight_time': 2,
'num_control_surfaces': 12,
'saturation_control_deflection_rate': np.deg2rad(50),
'get_gust': True,
'remove_rbm_integrals': True,
'ignore_elev': False, 
'get_thrust': False,
'n_nodes': 230,
'n_nodes_wing': 72,
'n_nodes_tail': 32,
'control_input_start':2780,
'gust_input':2779,
'thrust_input': [1]
}
with open(file_dir+'/parameter_state_space_{}.json'.format(case_name), 'w') as fp:
    json.dump(state_space_information, fp)

case_name ='superflexop_cruise_linear_ROM_free_flight'
state_space_information = {
    'aircraft_model': 'flexop',
    'case_name':case_name,
'u_inf': 45,
'rom_order': 4,
'num_modes': 30,
'num_aero_states': 108,
'rbm': True, 
'flight_time': 2,
'num_control_surfaces': 12,
'saturation_control_deflection_rate': np.deg2rad(50),
'get_gust': True,
'remove_rbm_integrals': True,
'ignore_elev': False, 
'get_thrust': False,
'n_nodes': 230,
'n_nodes_wing': 72,
'n_nodes_tail': 32,
'control_input_start':2780,
'gust_input':2779,
'thrust_input': [1]
}


with open(file_dir+'/parameter_state_space_{}.json'.format(case_name), 'w') as fp:
    json.dump(state_space_information, fp)

case_name ='superflexop_cruise_linear_ROM_clamped'
state_space_information = {
    'aircraft_model': 'flexop',
    'case_name':case_name,
'u_inf': 45,
'rom_order': 4,
'num_modes': 30,
'num_aero_states': 120,
'rbm': False, 
'flight_time': 2,
'num_control_surfaces': 12,
'saturation_control_deflection_rate': np.deg2rad(50),
'get_gust': True,
'remove_rbm_integrals': False,
'ignore_elev': False, 
'get_thrust': False,
'n_nodes': 230,
'n_nodes_wing': 72,
'n_nodes_tail': 32,
'control_input_start':2762,
'gust_input':2761,
'thrust_input': [1]
}


with open(file_dir+'/parameter_state_space_{}.json'.format(case_name), 'w') as fp:
    json.dump(state_space_information, fp)


case_name ='superflexop_cruise_linear_FOM_clamped'
state_space_information = {
    'aircraft_model': 'flexop',
    'case_name':case_name,
'u_inf': 45,
'rom_order': 0,
'num_modes': 30,
'num_aero_states': 120,
'rbm': False, 
'flight_time': 2,
'num_control_surfaces': 12,
'saturation_control_deflection_rate': np.deg2rad(50),
'get_gust': True,
'remove_rbm_integrals': False,
'ignore_elev': False, 
'get_thrust': False,
'n_nodes': 230,
'n_nodes_wing': 72,
'n_nodes_tail': 32,
'control_input_start':2762,
'gust_input':2761,
'thrust_input': [1]
}


with open(file_dir+'/parameter_state_space_{}.json'.format(case_name), 'w') as fp:
    json.dump(state_space_information, fp)
