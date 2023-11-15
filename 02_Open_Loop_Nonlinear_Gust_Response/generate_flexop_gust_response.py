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

# Define folder
file_dir = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
cases_route = '../../cases/'


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
    'gust_offset': 500,
    # Discretisation
    'mstar': 80,  # Number of streamwise wake panels
    'num_chord_panels': 8,  # Chordwise lattice discretization
    'n_elem_multiplier': 2,  # Multiplier for spanwise node discretization
    # Others
    'n_tstep': 2700,  # Number of simulation time steps
    'num_cores': 4,  # Number of CPU cores used for parallelization
    'sigma': 0.3,  # Stiffness scaling factor (1 for FLEXOP, 0.3 for SuperFLEXOP)
    'dynamic_cs_input': False, # True if pre-defined control surface deflection used
    'dynamic_cs_input_file': file_dir + '/predefined_cs_inputs/linear_LQG_L10_I10.txt', # File containing the pre-defined control surface deflection inputs
    'postprocessors_dynamic': ['BeamLoads', 'SaveData', 'BeamPlot', 'AerogridPlot'],
    # Restart/ Pickle options
    'restart_case': True,
    'restart_pickle_file': file_dir + '/../lib/sharpy/output/superflexop_free_gust_comp2_L_10_I_10_p_0_f_0_cfl_1_uinf45//superflexop_free_gust_comp2_L_10_I_10_p_0_f_0_cfl_1_uinf45.pkl',# None,
    'save_pickle_file': True,
}

# Set initial aircraft trim values
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
        if simulation_settings['lateral_gust']:
            gust_settings['gust_component']= [1]
        elif simulation_settings['3D_velocity_component']:
            gust_settings['gust_component']= [0, 1, 2]

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
            'gust_component': 2,
        }
        if simulation_settings["lateral_gust"]:
            gust_settings['gust_component']= 1

else:
    gust_settings = {'use_gust':False}

# Set pre-defined control surface inputs 
if simulation_settings["dynamic_cs_input"]:
    route_dir = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))

    dict_predefined_cs_input_files = {
        '0' : simulation_settings["dynamic_cs_input_file"], # aileron 1 (inboard right)
        '1' : simulation_settings["dynamic_cs_input_file"], # aileron 2
        '2' : simulation_settings["dynamic_cs_input_file"], # aileron 3
        '3' : simulation_settings["dynamic_cs_input_file"],# aileron 4 (outboard right)
        '4' : None, # elevator 1 (inboard right)
        '5' : None, # elevator 2 (outboard right)
        '6' : simulation_settings["dynamic_cs_input_file"], # aileron 5 (inboard left)
        '7' : simulation_settings["dynamic_cs_input_file"], # aileron 6
        '8' : simulation_settings["dynamic_cs_input_file"], # aileron 7
        '9' :  simulation_settings["dynamic_cs_input_file"], # aileron 8 (outboard left)
        '10' : None, # elevator 3 (inboard left)
        '11' : None, # elevator 4 (outboard left))
    }
    ailerons_type = 1
else:
    ailerons_type = 0
    dict_predefined_cs_input_files = {}

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
if not simulation_settings['restart_case']:
    flow = [
        'BeamLoader',
        'AerogridLoader',
        'NonliftingbodygridLoader',
        # 'AerogridPlot',
        'BeamPlot',
        'StaticCoupled',
        'StaticTrim',
        'DynamicCoupled',
    ]

    # Remove certain steps based on simulation settings
    if simulation_settings['lifting_only']:
        flow.remove('NonliftingbodygridLoader')
    if simulation_settings['use_trim']:
        flow.remove('StaticCoupled')
    else:
        flow.remove('StaticTrim')
else:
    flow = ['DynamicCoupled']
if simulation_settings['save_pickle_file']:
    flow.append('PickleData')

# Loop over various gust lengths
list_gust_lengths = [10]  # List of gust lengths to simulate

for gust_length in list_gust_lengths:
    gust_settings['gust_length'] = gust_length

    # Generate a case name based on simulation settings
    if simulation_settings["continuous_gust"]:
        str_vel_components = ''
        for component in gust_settings['gust_component']:
            str_vel_components += str(component)
        case_name = 'superflexop_free_gust_continuous_comp{}_p_{}_f_{}_cfl_{}_uinf{}'.format(
            str_vel_components,
            int(simulation_settings['use_polars']),
            int(not simulation_settings['lifting_only']),
            int(not simulation_settings['wake_discretisation']),
            int(u_inf)
        )
    else:        
        case_name = 'superflexop_free_gust_comp{}_L_{}_I_{}_p_{}_f_{}_cfl_{}_uinf{}'.format(
            gust_settings['gust_component'],
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
    
    if not simulation_settings['restart_case']:
        case_name += '_restart'
    # Generate the FlexOP model and start the simulation
    flexop_model = generate_flexop_case(
        u_inf,
        rho,
        flow,
        initial_trim_values,
        case_name,
        cases_route=cases_route,
        gust_settings=gust_settings,
        dict_wake_shape=dict_wake_shape,
        **simulation_settings,
        ailerons_type=ailerons_type,
        dict_predefined_cs_input_files=dict_predefined_cs_input_files,
        nonlifting_interactions=bool(not simulation_settings["lifting_only"])
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

