import numpy as np
import sharpy.utils.algebra as algebra
import json
import os

_TOLERANCE                   = 1e-6
_FSI_TOLERANCE               = 1e-4
_STRUCTURAL_RELAXATION_FACTOR = 0.6
_RELAXATION_FACTOR           = 0.0
_NEWMARK_DAMP                = 0.5e-4
_N_LOAD_STEPS                = 5


def get_static_settings(flexop_model, u_inf, rho, alpha, cs_deflection, thrust,
                         gravity=True,
                         horseshoe=False,
                         variable_wake=False,
                         num_cores=2,
                         tolerance=_TOLERANCE,
                         fsi_tolerance=_FSI_TOLERANCE,
                         n_load_steps=_N_LOAD_STEPS,
                         structural_relaxation_factor=_STRUCTURAL_RELAXATION_FACTOR,
                         nonlifting_body_interactions=False):
    """Build settings for the static solver stack.

    Returns:
        dict: Settings for NonLinearStatic, StaticUvlm, StaticCoupled, StaticTrim.
    """
    settings = {}

    settings['NonLinearStatic'] = {
        'print_info': 'off',
        'max_iterations': 150,
        'num_load_steps': 1,
        'delta_curved': 1e-1,
        'min_delta': tolerance,
        'gravity_on': gravity,
        'gravity': 9.81,
    }

    settings['StaticUvlm'] = {
        'print_info': 'on',
        'horseshoe': horseshoe,
        'num_cores': num_cores,
        'n_rollup': 0,
        'velocity_field_generator': 'SteadyVelocityField',
        'velocity_field_input': {
            'u_inf': u_inf,
            'u_inf_direction': [1., 0, 0],
        },
        'rho': rho,
        'cfl1': bool(not variable_wake),
        'nonlifting_body_interactions': nonlifting_body_interactions,
    }

    settings['StaticCoupled'] = {
        'print_info': 'off',
        'structural_solver': 'NonLinearStatic',
        'structural_solver_settings': settings['NonLinearStatic'],
        'aero_solver': 'StaticUvlm',
        'aero_solver_settings': settings['StaticUvlm'],
        'max_iter': 100,
        'n_load_steps': n_load_steps,
        'tolerance': fsi_tolerance,
        'relaxation_factor': structural_relaxation_factor,
        'nonlifting_body_interactions': nonlifting_body_interactions,
    }

    settings['StaticTrim'] = {
        'solver': 'StaticCoupled',
        'solver_settings': settings['StaticCoupled'],
        'initial_alpha': alpha,
        'initial_deflection': cs_deflection,
        'initial_thrust': thrust,
        'tail_cs_index': [4, 5, 10, 11],
        'thrust_nodes': [0],
        'fz_tolerance': 1e-10,
        'fx_tolerance': 1e-10,
        'm_tolerance': 1e-10,
        'max_iter': 200,
        'save_info': True,
    }

    return settings


def get_dynamic_settings(flexop_model, dt, u_inf, rho, n_tstep,
                          free_flight=True,
                          gravity=True,
                          num_cores=2,
                          newmark_damp=_NEWMARK_DAMP,
                          tolerance=_TOLERANCE,
                          fsi_tolerance=_FSI_TOLERANCE,
                          relaxation_factor=_RELAXATION_FACTOR,
                          variable_wake=False,
                          mstar=80,
                          dict_wake_shape=None,
                          gust_config=None,
                          dynamic_cs_input=False,
                          dict_predefined_cs_input_files=None,
                          postprocessors=None,
                          nonlifting_body_interactions=False,
                          restart_case=False,
                          include_unsteady_force_contribution=True):
    """Build settings for the dynamic solver stack.

    Returns:
        dict: Settings for AerogridLoader, NonliftingbodygridLoader,
            NonLinearDynamicCoupledStep, NonLinearDynamicPrescribedStep,
            StepUvlm, DynamicCoupled.
    """
    if postprocessors is None:
        postprocessors = ['BeamLoads', 'SaveData']
    if dict_predefined_cs_input_files is None:
        dict_predefined_cs_input_files = {}

    settings = {}

    # AerogridLoader
    settings['AerogridLoader'] = {
        'unsteady': 'on',
        'aligned_grid': 'on',
        'mstar': mstar,
        'wake_shape_generator': 'StraightWake',
        'wake_shape_generator_input': {
            'u_inf': u_inf,
            'u_inf_direction': [1., 0., 0.],
            'dt': dt,
        },
    }
    if variable_wake:
        print("mstar = ", settings['AerogridLoader']['mstar'])
        settings['AerogridLoader']['wake_shape_generator_input'] = dict_wake_shape or {
            'dx1': flexop_model.aero.chord_main_tip / flexop_model.aero.m,
            'ndx1': 23,
            'r': 1.6,
            'dxmax': 5 * flexop_model.aero.chord_main_root,
        }
    if dynamic_cs_input:
        settings['AerogridLoader']['control_surface_deflection'] = (
            [''] * flexop_model.aero.n_control_surfaces
        )
        settings['AerogridLoader']['control_surface_deflection_generator_settings'] = {}
        for i_cs in range(flexop_model.aero.n_control_surfaces):
            cs_file = dict_predefined_cs_input_files.get(str(i_cs))
            if cs_file is not None:
                settings['AerogridLoader']['control_surface_deflection_generator_settings'][str(i_cs)] = {
                    'dt': dt,
                    'deflection_file': cs_file,
                }
                settings['AerogridLoader']['control_surface_deflection'][i_cs] = 'DynamicControlSurface'

    settings['NonliftingbodygridLoader'] = {}

    settings['NonLinearDynamicCoupledStep'] = {
        'print_info': 'off',
        'max_iterations': 950,
        'delta_curved': 1e-1,
        'min_delta': tolerance,
        'newmark_damp': newmark_damp,
        'gravity_on': gravity,
        'gravity': 9.81,
        'num_steps': n_tstep,
        'dt': dt,
        'initial_velocity': u_inf,
    }

    settings['NonLinearDynamicPrescribedStep'] = {
        'print_info': 'off',
        'max_iterations': 950,
        'delta_curved': 1e-1,
        'min_delta': tolerance,
        'newmark_damp': newmark_damp,
        'gravity_on': gravity,
        'gravity': 9.81,
        'num_steps': n_tstep,
        'dt': dt,
    }

    settings['StepUvlm'] = {
        'num_cores': num_cores,
        'convection_scheme': 2,
        'gamma_dot_filtering': 7,
        'cfl1': bool(not variable_wake),
        'velocity_field_generator': 'SteadyVelocityField',
        'velocity_field_input': {
            'u_inf': u_inf * int(not free_flight),
            'u_inf_direction': [1., 0, 0],
        },
        'rho': rho,
        'n_time_steps': n_tstep,
        'dt': dt,
        'nonlifting_body_interactions': nonlifting_body_interactions,
    }
    if gust_config is not None:
        settings['StepUvlm']['velocity_field_generator'] = 'GustVelocityField'
        settings['StepUvlm']['velocity_field_input'] = {
            'u_inf': u_inf,
            'u_inf_direction': [1., 0, 0],
            'relative_motion': bool(not free_flight),
            'offset': gust_config.gust_offset,
            'gust_shape': gust_config.gust_shape,
        }
        if gust_config.gust_shape == 'time varying':
            settings['StepUvlm']['velocity_field_input']['gust_parameters'] = {
                'file': gust_config.file,
                'gust_component': gust_config.gust_component,
            }
        else:
            settings['StepUvlm']['velocity_field_input']['gust_parameters'] = {
                'gust_length': gust_config.gust_length,
                'gust_intensity': gust_config.gust_intensity * u_inf,
                'gust_component': gust_config.gust_component,
            }

    structural_solver = (
        'NonLinearDynamicCoupledStep' if free_flight
        else 'NonLinearDynamicPrescribedStep'
    )
    settings['DynamicCoupled'] = {
        'structural_solver': structural_solver,
        'structural_solver_settings': settings[structural_solver],
        'aero_solver': 'StepUvlm',
        'aero_solver_settings': settings['StepUvlm'],
        'fsi_substeps': 200,
        'fsi_tolerance': fsi_tolerance,
        'relaxation_factor': relaxation_factor,
        'minimum_steps': 1,
        'relaxation_steps': 150,
        'final_relaxation_factor': 0.05,
        'n_time_steps': n_tstep,
        'dt': dt,
        'include_unsteady_force_contribution': include_unsteady_force_contribution,
        'postprocessors': postprocessors,
        'postprocessors_settings': {},
        'nonlifting_body_interactions': nonlifting_body_interactions,
    }
    if restart_case:
        settings['DynamicCoupled']['relaxation_factor'] = 0.
        settings['DynamicCoupled']['final_relaxation_factor'] = 0.

    return settings


def get_linear_settings(flexop_model, dt, u_inf, rho, num_modes,
                         free_flight=True,
                         gravity=True,
                         newmark_damp=_NEWMARK_DAMP,
                         recover_accelerations=False,
                         remove_gust_input=False,
                         rom_settings=None,
                         flow=None):
    """Build settings for the linearisation solver stack.

    Returns:
        dict: Settings for Modal and LinearAssembler.
    """
    flow = flow or []
    settings = {}

    settings['Modal'] = {
        'print_info': True,
        'use_undamped_modes': True,
        'NumLambda': num_modes,
        'rigid_body_modes': free_flight,
        'write_modes_vtk': 'on',
        'print_matrices': 'on',
        'continuous_eigenvalues': 'off',
        'dt': dt,
        'plot_eigenvalues': False,
    }

    settings['LinearAssembler'] = {
        'linear_system': 'LinearAeroelastic',
        'inout_coordinates': 'nodes',
        'recover_accelerations': recover_accelerations,
        'linear_system_settings': {
            'track_body': free_flight,
            'use_euler': free_flight,
            'beam_settings': {
                'modal_projection': True,
                'inout_coords': 'modes',
                'discrete_time': True,
                'newmark_damp': newmark_damp,
                'discr_method': 'newmark',
                'dt': dt,
                'proj_modes': 'undamped',
                'num_modes': num_modes,
                'print_info': 'on',
                'gravity': gravity,
            },
            'aero_settings': {
                'dt': dt,
                'integr_order': 2,
                'density': rho,
                'remove_predictor': True,
                'use_sparse': 'off',
                'gust_assembler': 'LeadingEdge',
            },
        },
    }

    if remove_gust_input:
        settings['LinearAssembler']['linear_system_settings']['aero_settings']['remove_inputs'] = ['u_gust']

    if rom_settings is not None and rom_settings.use:
        aero = settings['LinearAssembler']['linear_system_settings']['aero_settings']
        aero['rom_method'] = rom_settings.rom_method
        aero['rom_method_settings'] = rom_settings.rom_method_settings

    if 'AsymptoticStability' in flow:
        aero = settings['LinearAssembler']['linear_system_settings']['aero_settings']
        aero['ScalingDict'] = {
            'length': flexop_model.aero.chord_main_root / 2,
            'speed': u_inf,
            'density': rho,
        }
        aero['remove_inputs'] = ['u_gust']

    return settings


def get_settings(flexop_model, flow, dt, u_inf, rho, alpha, cs_deflection, thrust,
                  aircraft_config=None, run_config=None,
                  gust_config=None, rom_settings=None):
    """Assemble the full SHARPy settings dictionary.

    Calls get_static_settings, get_dynamic_settings, and (when LinearAssembler
    is in the flow) get_linear_settings, then appends output/postprocessor
    settings and wires up DynamicCoupled.postprocessors_settings.

    Args:
        flexop_model: Configured FLEXOP model instance.
        flow: Ordered list of SHARPy solver names.
        dt: Time step [s].
        u_inf: Free-stream speed [m/s].
        rho: Air density [kg/m^3].
        alpha: Angle of attack [rad].
        cs_deflection: Initial control-surface deflection [rad].
        thrust: Engine thrust [N].
        aircraft_config: Aircraft and discretisation parameters.
        run_config: Solver and execution parameters.
        gust_config: Gust parameters, or None for no gust.
        rom_settings: ROM parameters, or None.

    Returns:
        dict: Full SHARPy settings dictionary.
    """
    from flexop_simulation_config import AircraftConfig, RunConfig
    aircraft_config = aircraft_config or AircraftConfig()
    run_config      = run_config      or RunConfig()

    # --- extract from aircraft_config ---
    gravity                      = aircraft_config.gravity
    horseshoe                    = aircraft_config.horseshoe
    variable_wake                = aircraft_config.variable_wake
    nonlifting_body_interactions = aircraft_config.nonlifting_interactions
    mstar                        = aircraft_config.mstar
    dict_wake_shape              = aircraft_config.dict_wake_shape
    use_polars                   = aircraft_config.use_polars

    # --- extract from run_config ---
    num_cores                      = run_config.num_cores
    n_tstep                        = run_config.n_tstep
    free_flight                    = run_config.free_flight
    num_modes                      = run_config.num_modes
    recover_accelerations          = run_config.recover_accelerations
    remove_gust_input              = run_config.remove_gust_input_in_statespace
    postprocessors_dynamic         = run_config.postprocessors_dynamic
    dynamic_cs_input               = run_config.dynamic_cs_input
    dict_predefined_cs_input_files = run_config.dict_predefined_cs_input_files or {}
    restart_case                   = run_config.restart_case

    include_unsteady_force = 'LinearAssembler' not in flow

    # --- base settings ---
    settings = {}
    settings['SHARPy'] = {
        'case':         flexop_model.case_name,
        'route':        flexop_model.case_route,
        'flow':         flow,
        'write_screen': 'on',
        'write_log':    'on',
        'log_folder':   flexop_model.output_route,
        'log_file':     flexop_model.case_name + '.log',
    }
    settings['BeamLoader'] = {
        'unsteady':    'on',
        'orientation': algebra.euler2quat(np.array([0., alpha, 0.])),
    }

    # --- static ---
    settings.update(get_static_settings(
        flexop_model, u_inf, rho, alpha, cs_deflection, thrust,
        gravity=gravity,
        horseshoe=horseshoe,
        variable_wake=variable_wake,
        num_cores=num_cores,
        nonlifting_body_interactions=nonlifting_body_interactions,
    ))

    # --- dynamic ---
    settings.update(get_dynamic_settings(
        flexop_model, dt, u_inf, rho, n_tstep,
        free_flight=free_flight,
        gravity=gravity,
        num_cores=num_cores,
        variable_wake=variable_wake,
        mstar=mstar,
        dict_wake_shape=dict_wake_shape,
        gust_config=gust_config,
        dynamic_cs_input=dynamic_cs_input,
        dict_predefined_cs_input_files=dict_predefined_cs_input_files,
        postprocessors=postprocessors_dynamic,
        nonlifting_body_interactions=nonlifting_body_interactions,
        restart_case=restart_case,
        include_unsteady_force_contribution=include_unsteady_force,
    ))

    # --- output / postprocessor settings ---
    settings['SaveData'] = {
        'save_aero':   True,
        'save_struct': True,
    }
    settings['BeamPlot']  = {}
    settings['BeamLoads'] = {'csv_output': True}
    settings['AerogridPlot'] = {
        'include_rbm':            'off',
        'include_applied_forces': 'on',
        'minus_m_star':           5,
        'u_inf':                  u_inf,
        'plot_nonlifting_surfaces': nonlifting_body_interactions,
    }
    settings['AeroForcesCalculator'] = {
        'write_text_file': 'on',
        'nonlifting_body': nonlifting_body_interactions,
        'coefficients':    'off',
        'S_ref':           flexop_model.reference_area,
        'q_ref':           0.5 * rho * u_inf ** 2,
    }
    settings['PickleData'] = {}
    settings['WriteVariablesTime'] = {
        'structure_variables':    ['pos', 'psi'],
        'structure_nodes':        list(range(flexop_model.structure.n_node_main + 1)),
        'cleanup_old_solution':   'on',
        'delimiter':              ',',
    }
    settings['AsymptoticStability'] = {
        'print_info':        'on',
        'frequency_cutoff':  0,
        'export_eigenvalues': 'on',
        'modes_to_plot':     num_modes,
        'velocity_analysis': [30, 60, 7],
    }
    settings['LiftDistribution']   = {'rho': rho}
    settings['SaveParametricCase'] = {
        'parameters': {'alpha': np.rad2deg(alpha), 'u_inf': u_inf},
        'save_case':  'off',
    }

    # --- linear (optional) ---
    if 'LinearAssembler' in flow:
        settings.update(get_linear_settings(
            flexop_model, dt, u_inf, rho, num_modes,
            free_flight=free_flight,
            gravity=gravity,
            recover_accelerations=recover_accelerations,
            remove_gust_input=remove_gust_input,
            rom_settings=rom_settings,
            flow=flow,
        ))
        settings['SaveData']['save_linear'] = True
        if rom_settings is not None and rom_settings.use:
            settings['SaveData']['save_rom'] = True

    # --- wire up postprocessors_settings ---
    for pp in settings['DynamicCoupled']['postprocessors']:
        settings['DynamicCoupled']['postprocessors_settings'][pp] = settings.get(pp, {})

    if use_polars:
        settings = update_settings_for_polar_corrections(settings)

    return settings


def update_settings_for_polar_corrections(settings):
    """Add polar-correction force settings to StaticCoupled and DynamicCoupled.

    Args:
        settings: Full SHARPy settings dictionary (modified in place).

    Returns:
        dict: Updated settings dictionary.
    """
    print("update polar settings!")
    aoa_cl_deg = [-3.28415340783741, 0]
    for solver in ['StaticCoupled', 'DynamicCoupled']:
        settings[solver]['correct_forces_method'] = 'PolarCorrection'
        settings[solver]['correct_forces_settings'] = {
            'cd_from_cl':       'off',
            'correct_lift':     'on',
            'moment_from_polar': 'on',
            'skip_surfaces':    [],
            'aoa_cl0':          np.deg2rad(aoa_cl_deg),
            'write_induced_aoa': False,
        }
    return settings


def get_skipped_attributes(list_to_be_saved_attr):
    """Return attributes from the full list that are not in list_to_be_saved_attr.

    Args:
        list_to_be_saved_attr: Attributes that should be kept.

    Returns:
        list: Attributes to skip (i.e. the complement of list_to_be_saved_attr).
    """
    route_dir = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
    with open(route_dir + '/list_aero_and_structural_ts_attributes.json', 'r') as f:
        list_of_all_attributes = json.load(f)
    for attribute in list_to_be_saved_attr:
        list_of_all_attributes.remove(attribute)
    return list_of_all_attributes
