"""
SuperFLEXOP open-loop nonlinear gust response simulation.

Runs a nonlinear aeroelastic simulation for a 1-cosine or continuous
(time-varying) gust and writes SHARPy case files and output to disk.

Usage (command line)::

    python generate_flexop_gust_response.py

Usage (import)::

    from generate_flexop_gust_response import run_gust_response
    run_gust_response(gust_length=15.0, gust_intensity=0.05)
"""

import dataclasses
import os
import sys
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../01_Aircraft_Model_Generator'))
from generate_flexop_case import generate_flexop_case
from flexop_simulation_config import (
    U_INF, RHO,
    AircraftConfig, RunConfig, TrimValues, GustConfig,
)

_FILE_DIR    = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
_CASES_ROUTE = os.path.join(_FILE_DIR, '../cases/')

_TRIM_VALUES = TrimValues(
    alpha=  6.406771329255241468e-03,   # angle of attack [rad]
    delta= -3.325087601649625961e-03,   # elevator deflection [rad]
    thrust= 2.052055145318664842e+00,   # engine thrust [N]
)

_AIRCRAFT_CONFIG = AircraftConfig(
    # all structural/aero defaults are inherited from AircraftConfig;
    # only values that differ from the class defaults are listed here
    horseshoe=False,
    use_polars=False,
    variable_wake=False,
)

_RUN_CONFIG = RunConfig(
    n_tstep=2700,
    num_cores=4,
    free_flight=True,
    postprocessors_dynamic=['BeamLoads', 'SaveData', 'BeamPlot', 'AerogridPlot'],
    dynamic_cs_input=False,
    dynamic_cs_input_file=os.path.join(_FILE_DIR, 'predefined_cs_inputs/linear_LQG_L10_I10.txt'),
    restart_case=False,
    restart_pickle_file=None,
    save_pickle_file=False,
)

# Continuous gust input file (only needed when continuous_gust=True)
_CONTINUOUS_GUST_FILE = os.path.join(
    _FILE_DIR,
    '../05_Utils/gust_inputs/'
    'turbulence_time_600s_uinf_45_altitude_800_moderate_noise_seeds_12782_12783_12784_12785_1.txt'
)


def _build_gust_config(continuous, gust_length, gust_intensity,
                        num_chord_panels, gust_offset_panels,
                        gust_input_file, lateral, three_d) -> GustConfig:
    """Return a GustConfig for generate_flexop_case.

    Args:
        continuous: Use time-varying gust instead of 1-cosine.
        gust_length: Gust wavelength [m] (1-cos only).
        gust_intensity: Intensity as fraction of U_INF (1-cos only).
        num_chord_panels: Number of chordwise aero panels (used to convert offset).
        gust_offset_panels: Gust start offset in chord panels upstream.
        gust_input_file: Path to time-series gust file (continuous only).
        lateral: Use lateral (y) gust component.
        three_d: Use all three velocity components.

    Returns:
        GustConfig: Configured gust settings.

    Raises:
        ValueError: If continuous=True and gust_input_file is missing or does not exist.
    """
    chord  = 0.471  # main root chord [m]
    offset = gust_offset_panels * chord / num_chord_panels

    if continuous:
        if gust_input_file is None or not os.path.exists(gust_input_file):
            raise ValueError(
                "A valid gust_input_file is required for continuous gust. "
                f"Got: {gust_input_file}"
            )
        if lateral:
            component = [1]
        elif three_d:
            component = [0, 1, 2]
        else:
            component = [2]
        return GustConfig(
            gust_shape='time varying',
            gust_offset=offset,
            gust_component=component,
            file=gust_input_file,
        )
    else:
        return GustConfig(
            gust_shape='1-cos',
            gust_offset=offset,
            gust_component=1 if lateral else 2,
            gust_length=gust_length,
            gust_intensity=gust_intensity,
        )


def _build_flow(use_trim, restart, save_pickle, lifting_only):
    """Return the ordered SHARPy solver flow list.

    Args:
        use_trim: Use StaticTrim instead of StaticCoupled.
        restart: Start from a pickled checkpoint, skipping static solvers.
        save_pickle: Append PickleData to the flow.
        lifting_only: Exclude NonliftingbodygridLoader.

    Returns:
        list[str]: Ordered SHARPy solver names.
    """
    if restart:
        flow = ['DynamicCoupled']
    else:
        flow = [
            'BeamLoader',
            'AerogridLoader',
            'NonliftingbodygridLoader',
            'BeamPlot',
            'StaticCoupled',
            'StaticTrim',
            'DynamicCoupled',
        ]
        if lifting_only:
            flow.remove('NonliftingbodygridLoader')
        if use_trim:
            flow.remove('StaticCoupled')
        else:
            flow.remove('StaticTrim')
    if save_pickle:
        flow.append('PickleData')
    return flow


def _build_case_name(gust_config: GustConfig, ac: AircraftConfig,
                      rc: RunConfig, continuous: bool) -> str:
    """Derive a descriptive case name from the simulation parameters.

    Args:
        gust_config: Gust configuration.
        ac: Aircraft configuration.
        rc: Run configuration.
        continuous: Whether a time-varying gust is used.

    Returns:
        str: Unique descriptive case name.
    """
    if continuous:
        components = ''.join(str(c) for c in gust_config.gust_component)
        name = 'superflexop_free_gust_continuous_comp{}_p_{}_f_{}_cfl_{}_uinf{}'.format(
            components,
            int(ac.use_polars),
            int(not ac.lifting_only),
            int(not ac.variable_wake),
            int(U_INF),
        )
    else:
        name = 'superflexop_free_gust_comp{}_L_{}_I_{}_p_{}_f_{}_cfl_{}_uinf{}'.format(
            gust_config.gust_component,
            gust_config.gust_length,
            int(gust_config.gust_intensity * 100),
            int(ac.use_polars),
            int(not ac.lifting_only),
            int(not ac.variable_wake),
            int(U_INF),
        )
    if ac.wing_only:
        name += '_wing_only'
    if rc.dynamic_cs_input:
        name += '_dynamic_cs_input'
    if rc.restart_case:
        name += '_restart'
    return name


def _build_cs_input_files(cs_file):
    """Map each aileron index to cs_file and elevator indices to None.

    Args:
        cs_file: Path to the control surface deflection time-series file.

    Returns:
        dict[str, str | None]: Mapping from control surface index (as string) to file path.
    """
    aileron_indices  = [0, 1, 2, 3, 6, 7, 8, 9]   # ailerons (right + left)
    elevator_indices = [4, 5, 10, 11]               # elevators
    return {
        **{str(i): cs_file for i in aileron_indices},
        **{str(i): None    for i in elevator_indices},
    }


def run_gust_response(
    gust_length: float = 10.0,
    gust_intensity: float = 0.1,
    continuous_gust: bool = False,
    gust_offset_panels: int = 500,
    lateral_gust: bool = False,
    three_d_gust: bool = False,
    gust_input_file: Optional[str] = None,
    aircraft_config: Optional[AircraftConfig] = None,
    run_config: Optional[RunConfig] = None,
):
    """Run a SuperFLEXOP nonlinear gust response simulation.

    Args:
        gust_length: Gust length for 1-cos gust [m]. Ignored for continuous gust.
        gust_intensity: Gust intensity as a fraction of U_INF (e.g. 0.1 = 10%).
        continuous_gust: Use a time-varying gust from file instead of 1-cosine.
        gust_offset_panels: Gust start offset expressed as number of chord panels upstream.
        lateral_gust: Use lateral (y) gust component instead of vertical (z).
        three_d_gust: Use all three velocity components. Overrides lateral_gust.
        gust_input_file: Path to the time-series gust file. Required when continuous_gust=True.
        aircraft_config: Full aircraft configuration. Defaults to _AIRCRAFT_CONFIG.
            Use dataclasses.replace(_AIRCRAFT_CONFIG, mstar=60) for partial overrides.
        run_config: Full run configuration. Defaults to _RUN_CONFIG.

    Returns:
        Configured and executed FLEXOP model.
    """
    ac = aircraft_config or _AIRCRAFT_CONFIG
    rc = run_config      or _RUN_CONFIG

    # Variable wake: update mstar and supply wake shape parameters
    if ac.variable_wake:
        chord = 0.471
        ac = dataclasses.replace(ac,
            mstar=35,
            dict_wake_shape={
                'dx1':   chord / ac.num_chord_panels,
                'ndx1':  23,
                'r':     1.6,
                'dxmax': 5 * chord,
            },
        )

    # Pre-defined control surface inputs
    if rc.dynamic_cs_input:
        rc = dataclasses.replace(rc,
            dict_predefined_cs_input_files=_build_cs_input_files(rc.dynamic_cs_input_file),
        )
        ac = dataclasses.replace(ac, ailerons_type=1)
    else:
        ac = dataclasses.replace(ac, ailerons_type=0)

    # Non-lifting body interactions: derive from lifting_only
    ac = dataclasses.replace(ac, nonlifting_interactions=not ac.lifting_only)

    gust_config = _build_gust_config(
        continuous=continuous_gust,
        gust_length=gust_length,
        gust_intensity=gust_intensity,
        num_chord_panels=ac.num_chord_panels,
        gust_offset_panels=gust_offset_panels,
        gust_input_file=gust_input_file or _CONTINUOUS_GUST_FILE,
        lateral=lateral_gust,
        three_d=three_d_gust,
    )

    flow = _build_flow(
        use_trim=True,
        restart=rc.restart_case,
        save_pickle=rc.save_pickle_file,
        lifting_only=ac.lifting_only,
    )

    case_name = _build_case_name(gust_config, ac, rc, continuous=continuous_gust)

    model = generate_flexop_case(
        U_INF, RHO, flow, _TRIM_VALUES, case_name,
        cases_route=_CASES_ROUTE,
        aircraft_config=ac,
        run_config=rc,
        gust_config=gust_config,
    )

    if not rc.restart_case:
        model.run()
    else:
        assert rc.restart_pickle_file is not None, \
            "restart_pickle_file must be set in run_config when restart_case=True"
        sys.path.insert(0, os.path.join(_FILE_DIR, '../05_Utils'))
        import restart_simulation_from_pickle as restart
        restart.restart_simulation_from_pickle(
            _CASES_ROUTE, case_name, rc.restart_pickle_file
        )

    return model


if __name__ == '__main__':
    # Default: single 1-cosine gust, length 10 m, intensity 10 % of U_INF
    run_gust_response()
