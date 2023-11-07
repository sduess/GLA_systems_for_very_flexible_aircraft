"""
(Super)FLEXOP Gust Response Simulation Script

This script sets up and runs a FlexOP simulation for analyzing the aeroelastic response of 
a flexible aircraft to a gust. 

Usage:
- Modify the simulation_settings, initial_trim_values, and gust_settings to match your specific case.
- Run the script to perform the FlexOP Simulation.

"""

# Import necessary modules
import sys, os
sys.path.insert(1,'../../01_Aircraft_Model_Generator')
from generate_flexop_case import generate_flexop_case
file_dir = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))


# Define simulation parameters

u_inf = 45  # Cruise flight speed in m/s
rho = 1.1336  # Air density in kg/m^3 (corresponding to an altitude of 800m)
alpha_rad = 6.406771329255241468e-03  # Angle of attack in radians (approximately 0.389 degrees)

# Simulation settings
simulation_settings = {
    'lifting_only': True,  # Ignore nonlifting bodies
    'wing_only': False,  # Simulate the full configuration (wing+tail)
    'dynamic': True,  # Perform unsteady simulation
    'wake_discretisation': False,  # Use variable wake discretization scheme
    'gravity': True,  # Include gravitational effects
    'horseshoe': False,  # Disable horseshoe wake modeling
    'use_polars': False,  # Apply polar corrections
    'free_flight': True,  # Simulate unclamped aircraft
    'use_trim': False,  # Enable aircraft trim
    # Gust Settings
    'use_gust': True,  # 
    'continuous_gust': False,  # 
    'gust_input_file': os.path.join(file_dir, '../05_Utils/gust_inputs/turbulence_time_600s_uinf_45_altitude_800_moderate_noise_seeds_12782_12783_12784_12785_1.txt'),
    'lateral_gust': False,  # 
    '3D_velocity_component': False,
    # Discretisation
    'mstar': 80,  # Number of streamwise wake panels
    'num_chord_panels': 8,  # Chordwise lattice discretization
    'n_elem_multiplier': 2,  # Multiplier for spanwise node discretization
    # Others
    'n_tstep': 2000,  # Number of simulation time steps
    'num_cores': 4,  # Number of CPU cores used for parallelization
    'sigma': 0.3,  # Stiffness scaling factor (1 for FLEXOP, 0.3 for SuperFLEXOP)
    'postprocessors_dynamic': ['BeamLoads', 'SaveData', 'BeamPlot', 'AerogridPlot'],
}

# Initial trim values
initial_trim_values = {
    'alpha': alpha_rad,
    'delta': -3.325087601649625961e-03,
    'thrust': 2.052055145318664842e+00
}


# Set Gust settings
if simulation_settings["use_gust"]:
    if simulation_settings["continuous_gust"]:
        gust_settings = {
            'use_gust': True,  # Enable gust modeling
            'gust_shape': 'time varying',
            'file': simulation_settings['gust_input_file'], # File includes time series of gust velocities, i.e. 4 columns: time[s]  U_x U_y U_z
            'gust_offset': 10,
            'gust_component': [2], # list of velocity components considered (0: U_x, 1: U_y, 2: U_z)   
        }
        if simulation_settings['gust_input_file'] is None or \
            not os.path.exists(simulation_settings['gust_input_file']):
            raise "Please specify a valid gust input file!"
    else:
        gust_settings = {
            'use_gust': True,  # Enable gust modeling
            'gust_shape': '1-cos',  # Gust shape function
            'gust_length': 10.0,  # Gust length in seconds
            'gust_intensity': 0.1,  # Gust intensity
            'gust_offset': 10,
            'gust_component': [2],
        }
    if simulation_settings["lateral_gust"]:
        gust_settings['gust_component']= [1]
    elif simulation_settings["3D_velocity_component"]:
        gust_settings['gust_component']= [0, 1, 2]

else:
    gust_settings = {'use_gust':False}

# Set wake shape inputs if needed for variable wake discretization
if simulation_settings['wake_discretisation']:
    dict_wake_shape = {
        'dx1': 0.471 / simulation_settings['num_chord_panels'],
        'ndx1': 23,
        'r': 1.6,
        'dxmax': 5 * 0.471
    }
    simulation_settings['mstar'] = 35
else:
    dict_wake_shape = None

# Define the flow sequence
flow = [
    'BeamLoader',
    'AerogridLoader',
    'NonliftingbodygridLoader',
    # 'AerogridPlot',
    'BeamPlot',
    'StaticCoupled',
    'DynamicCoupled',
]

# Remove certain steps based on simulation settings
if simulation_settings['lifting_only']:
    flow.remove('NonliftingbodygridLoader')

# Loop over various gust lengths
list_gust_lengths = [10]  # List of gust lengths to simulate

for gust_length in list_gust_lengths:
    gust_settings['gust_length'] = gust_length

    str_vel_components = ''
    for component in gust_settings['gust_component']:
        str_vel_components += str(component)
    # Generate a case name based on simulation settings
    if simulation_settings["continuous_gust"]:
        case_name = 'superflexop_free_gust_continuous_comp{}_p_{}_f_{}_cfl_{}_uinf{}'.format(
            str_vel_components,
            int(simulation_settings['use_polars']),
            int(not simulation_settings['lifting_only']),
            int(not simulation_settings['wake_discretisation']),
            int(u_inf)
        )
    else:        
        case_name = 'superflexop_free_gust_comp{}_L_{}_I_{}_p_{}_f_{}_cfl_{}_uinf{}'.format(
            str_vel_components,
            gust_settings['gust_length'],
            int(gust_settings['gust_intensity'] * 100),
            int(simulation_settings['use_polars']),
            int(not simulation_settings['lifting_only']),
            int(not simulation_settings['wake_discretisation']),
            int(u_inf)
        )
    
    # Include 'nonlifting' in the case name if nonlifting bodies are considered
    if not simulation_settings["lifting_only"]:
        case_name += '_nonlifting'
    
    # Generate the FlexOP model and start the simulation
    flexop_model = generate_flexop_case(
        u_inf,
        rho,
        flow,
        initial_trim_values,
        case_name,
        gust_settings=gust_settings,
        dict_wake_shape=dict_wake_shape,
        **simulation_settings,
        nonlifting_interactions=bool(not simulation_settings["lifting_only"])
    )
    flexop_model.run()