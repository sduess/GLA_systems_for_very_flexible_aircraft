import os
import flexop as aircraft
import numpy as np
from helper_functions.get_settings import get_settings
from flexop_simulation_config import AircraftConfig, RunConfig, TrimValues, GustConfig, RomSettings

_CFL = 1


def generate_flexop_case(u_inf,
                          rho,
                          flow,
                          trim_values: TrimValues,
                          case_name,
                          cases_route='../../cases/',
                          aircraft_config: AircraftConfig = None,
                          run_config: RunConfig = None,
                          gust_config: GustConfig = None,
                          rom_settings: RomSettings = None):
    """Generate and configure a FLEXOP aircraft model for a simulation case.

    Args:
        u_inf: Cruise flight speed [m/s].
        rho: Air density [kg/m^3].
        flow: Ordered list of SHARPy solver names.
        trim_values: Initial trim state (alpha, delta, thrust).
        case_name: Unique identifier for the simulation case.
        cases_route: Directory where case input files are written.
        aircraft_config: Aircraft and discretisation parameters. Defaults to AircraftConfig().
        run_config: Solver and execution parameters. Defaults to RunConfig().
        gust_config: Gust excitation parameters. Pass None for no gust.
        rom_settings: ROM parameters. Pass None to use the full-order model.

    Returns:
        Configured FLEXOP model ready to run.
    """
    aircraft_config = aircraft_config or AircraftConfig()
    run_config      = run_config      or RunConfig()

    alpha         = trim_values.alpha
    cs_deflection = trim_values.delta
    thrust        = trim_values.thrust

    # Polar data (only loaded when requested)
    data_polars = None
    if aircraft_config.use_polars:
        flexop_directory = os.path.abspath(aircraft.FLEXOP_DIRECTORY)
        airfoil_polars = {
            0: flexop_directory + '/src/airfoil_polars_alpha_30_/xfoil_seq_re1300000_root.txt',
            1: flexop_directory + '/src/airfoil_polars_alpha_30_/xfoil_seq_re1300000_naca0012.txt',
        }
        data_polars = generate_polar_arrays(airfoil_polars)

    # Build FLEXOP model
    flexop_model = aircraft.FLEXOP(case_name, cases_route, run_config.output_folder)
    flexop_model.clean()
    flexop_model.init_structure(
        sigma=aircraft_config.sigma,
        n_elem_multiplier=aircraft_config.n_elem_multiplier,
        n_elem_multiplier_fuselage=aircraft_config.n_elem_multiplier_fuselage,
        lifting_only=aircraft_config.lifting_only,
        wing_only=aircraft_config.wing_only,
        delta_x_payload=aircraft_config.delta_x_payload,
    )
    flexop_model.init_aero(
        m=aircraft_config.num_chord_panels,
        cs_deflection=cs_deflection,
        polars=data_polars,
        ailerons_type=aircraft_config.ailerons_type,
    )

    if aircraft_config.nonlifting_interactions:
        flexop_model.init_fuselage(m=aircraft_config.num_radial_panels)

    flexop_model.structure.set_thrust(thrust)
    flexop_model.generate()
    flexop_model.structure.calculate_aircraft_mass()

    dt = _CFL * flexop_model.aero.chord_main_root / flexop_model.aero.m / u_inf

    settings = get_settings(
        flexop_model, flow, dt, u_inf, rho, alpha, cs_deflection, thrust,
        aircraft_config=aircraft_config,
        run_config=run_config,
        gust_config=gust_config,
        rom_settings=rom_settings,
    )

    flexop_model.create_settings(settings)
    return flexop_model


def generate_polar_arrays(airfoils):
    """Load and return polar data arrays for each airfoil.

    Args:
        airfoils: Mapping from airfoil index to file path.

    Returns:
        list: Polar data arrays, one per airfoil.
    """
    out_data = [None] * len(airfoils)
    for airfoil_index, airfoil_filename in airfoils.items():
        out_data[airfoil_index] = np.loadtxt(airfoil_filename, skiprows=12)[:, :4]
        if any(out_data[airfoil_index][:, 0] > 1):
            out_data[airfoil_index][:, 0] *= np.pi / 180
    return out_data
