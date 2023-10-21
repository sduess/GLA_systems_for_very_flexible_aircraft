"""
(Super)FLEXOP Linearization Script

This script sets up and runs a FlexOP simulation and linearizes the model around its nonlinear 
equlibrium after a specified number of computed timesteps. The linearized model can than be
reduced using SHARPy's various order reduction methods. The linear full-order model (FOM) or 
linear reduced-order model (ROM) is exported to an h5-file output.

Usage:
- Modify the simulation_settings and initial_trim_values to match your specific case.
- Run the script to perform the FlexOP Simulation.

Next Steps:
- For using the linearized FOM/ROM for linear simulations or control design check out TODO add file locations.

"""

# Import necessary modules
import sys
sys.path.insert(1,'../../01_Aircraft_Model_Generator')
from generate_flexop_case import generate_flexop_case
import numpy as np


# Define folder
cases_route = '../../cases/'
case_name_string_format = 'superflexop_cruise_linear_{}_{}'
# Define simulation parameters
u_inf = 45  # Cruise flight speed in m/s
rho = 1.1336  # Air density in kg/m^3 (corresponding to an altitude of 800m)
alpha_rad = 6.630220785589852756e-03 # Angle of attack in radians (approximately 0.389 degrees)


2.284695456100073407e+00
# Simulation settings
simulation_settings = {
    'lifting_only': True,  # Ignore nonlifting bodies
    'wing_only': False,  # Simulate the full configuration (wing+tail)
    'wake_discretisation': False,  # Use variable wake discretization scheme
    'gravity': True,  # Include gravitational effects
    'free_flight': True,  # Simulate unclamped aircraft
    'use_trim': True, #True,  # Enable aircraft trim
    'use_rom': True,
    'restart_case': False,
    'restart_pickle_file': None,
    'remove_gust_input_in_statespace': False,
    'num_modes': 30,
    'mstar': 80,  # Number of streamwise wake panels
    'num_chord_panels': 8,  # Chordwise lattice discretization
    'n_elem_multiplier': 2,  # Multiplier for spanwise node discretization
    'n_elem_multiplier_fuselage': 1,  # Multiplier for longitudinal fuselage node discretization
    'n_tstep': 1,  # Number of simulation time steps
    'num_cores': 4,  # Number of CPU cores used for parallelization
    'sigma': 0.3,  # Stiffness scaling factor (1 for FLEXOP, 0.3 for SuperFLEXOP)
    'postprocessors_dynamic': [],
}

# Define case name
if simulation_settings['use_rom']:
    model_output = 'ROM' 
else:    
    model_output = 'FOM' 
if simulation_settings['free_flight']:
    rbm_conditions = 'free_flight'
else:    
    rbm_conditions = 'clamped'
case_name = case_name_string_format.format(model_output, rbm_conditions)

# Initial trim values
initial_trim_values = {
    'alpha': alpha_rad,
    'delta': -0.00405090919,
    'thrust': 2.284695456100073407e+00
}

# Define the flow sequence
if not simulation_settings['restart_case']:
    flow = [
        'BeamLoader', 
        'Modal',
        'AerogridLoader',
        'StaticCoupled',
        'StaticTrim',     
        'DynamicCoupled',
        'Modal',
        'LinearAssembler',
        'SaveData',
    ]
    if not simulation_settings['use_trim']:
        flow.remove('StaticTrim')
    else:
        flow.remove('StaticCoupled')
else:
    flow = [  
        'DynamicCoupled',
        'Modal',
        'LinearAssembler',
        'SaveData',
    ] 

# ROM settings
rom_settings = {
    'use': simulation_settings['use_rom'],
    'rom_method': ['Krylov'],
    'rom_method_settings': {'Krylov': {
                                        'algorithm': 'mimo_rational_arnoldi',
                                        'r': 4, 
                                        'frequency': np.array([0]),
                                        'single_side': 'observability',
                                        },
                           }
}


# Generate the FlexOP model and simulation settings
flexop_model = generate_flexop_case(
                        u_inf,
                        rho,
                        flow,
                        initial_trim_values,
                        case_name,
                        cases_route=cases_route,
                        rom_settings=rom_settings,
                        **simulation_settings
                        )

# Start simulation
if not simulation_settings['restart_case']:
    flexop_model.run()
else:
    assert simulation_settings['restart_pickle_file'] is not None, "Define a pickle file to restart from"
    
    sys.path.insert(1,'../../05_Utils')
    import restart_simulation_from_pickle as restart 
    restart.restart_simulation_from_pickle(cases_route, 
                                           case_name,
                                           simulation_settings['restart_pickle_file'])
