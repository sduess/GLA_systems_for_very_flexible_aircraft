"""
(Super)FLEXOP Asymptotic Stability Analysis

This script sets up and runs a(Super)FLEXOP

Usage:
-

"""

# Import necessary modules
import sys, os
sys.path.insert(1,'../../01_Aircraft_Model_Generator')
from generate_flexop_case import generate_flexop_case
import numpy as np

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
    'gravity': True,  # Include gravitational effects
    'horseshoe': False,  # Disable horseshoe wake modeling
    'use_polars': False,  # Apply polar corrections
    'free_flight': True,  # Simulate unclamped aircraft
    'use_trim': True,  # Enable aircraft trim
    'use_rom': True, # Use reduced-order model
    # Discretisation
    'mstar': 80,  # Number of streamwise wake panels
    'num_chord_panels': 8,  # Chordwise lattice discretization
    'n_elem_multiplier': 2,  # Multiplier for spanwise node discretization
    # Others
    'n_tstep': 100,  # Number of simulation time steps
    'num_cores': 4,  # Number of CPU cores used for parallelization
    'sigma': 0.3,  # Stiffness scaling factor (1 for FLEXOP, 0.3 for SuperFLEXOP)
}

# Set initial aircraft trim values
initial_trim_values = {
    'alpha': alpha_rad,
    'delta': -3.325087601649625961e-03,
    'thrust': 2.052055145318664842e+00
}

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

# Define the flow sequence
flow = ['BeamLoader',
        'AerogridLoader',
        'StaticTrim',
        'BeamPlot',
        'AerogridPlot',
        'AeroForcesCalculator',
        'DynamicCoupled',
        'Modal',
        'LinearAssembler',
        'AsymptoticStability',
        ]

# Generate a case name based on simulation settings

# Include 'nonlifting' in the case name if nonlifting bodies are considered
case_name = 'superflexop_asymptotic_stability_u_inf{}'.format(int(u_inf))
if not simulation_settings["lifting_only"]:
    case_name += '_nonlifting'

# Generate the FlexOP model and start the simulation
flexop_model = generate_flexop_case(
    u_inf,
    rho,
    flow,
    initial_trim_values,
    case_name,
    cases_route=cases_route,
    rom_settings=rom_settings,
    **simulation_settings,
    nonlifting_interactions=bool(not simulation_settings["lifting_only"])
)

flexop_model.run()
   