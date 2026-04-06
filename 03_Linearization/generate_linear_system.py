"""
SuperFLEXOP linearisation script.

Runs a nonlinear simulation to trim equilibrium, then linearises the model
around that equilibrium using SHARPy's LinearAssembler.  Optionally reduces
the resulting full-order model (FOM) to a ROM via Krylov subspace methods.

The linear state-space system is exported to an h5 file for downstream use
in postprocess_linear_system.m / get_linear_gust_response.m.

Usage (command line)::

    python generate_linear_system.py

Usage (import)::

    from generate_linear_system import run_linearization
    run_linearization(use_rom=False)    # produce FOM only
"""

import dataclasses
import os
import sys
from typing import Optional

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../01_Aircraft_Model_Generator'))
from generate_flexop_case import generate_flexop_case
from flexop_simulation_config import (
    U_INF, RHO,
    AircraftConfig, RunConfig, TrimValues, RomSettings,
)

_CASES_ROUTE = os.path.join(os.path.dirname(__file__), '../cases/')

_TRIM_VALUES = TrimValues(
    alpha=  6.630220785589852756e-03,   # angle of attack [rad]
    delta= -0.00405090919,              # elevator deflection [rad]
    thrust= 2.284695456100073407e+00,   # engine thrust [N]
)

_AIRCRAFT_CONFIG = AircraftConfig(mstar=80)

_RUN_CONFIG = RunConfig(
    n_tstep=500,                    # static trim does not perfectly trim the aircraft dynamically, thus a few more timesteps represent better equilibrium                                          
    num_cores=4,
    free_flight=True,               # free-flight vs clamped (rbm on/off)
    num_modes=30,                   # modal convergence study
    recover_accelerations=True,     # acceleration recovery not yet implemented, is done outside
    remove_gust_input_in_statespace=False,
    postprocessors_dynamic=[],
    restart_case=False,
    restart_pickle_file=None,
)

_ROM_SETTINGS = RomSettings(
    use=True,
    rom_method=['Krylov'],
    rom_method_settings={
        'Krylov': {
            'algorithm':   'mimo_rational_arnoldi',
            'r':           4,
            'frequency':   np.array([0]),
            'single_side': 'observability',
        },
    },
)


def _build_flow(use_trim, restart):
    """Return the ordered SHARPy solver flow list.

    Args:
        use_trim: Use StaticTrim instead of StaticCoupled.
        restart: Start from a pickled checkpoint, skipping static solvers.

    Returns:
        list[str]: Ordered SHARPy solver names.
    """
    if restart:
        return ['DynamicCoupled', 'Modal', 'LinearAssembler', 'SaveData']
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
    if use_trim:
        flow.remove('StaticCoupled')
    else:
        flow.remove('StaticTrim')
    return flow


def _build_case_name(use_rom: bool, free_flight: bool,
                      wing_only: bool, recover_accelerations: bool) -> str:
    """Derive a descriptive case name from the simulation parameters.

    Args:
        use_rom: Whether a ROM is produced.
        free_flight: Whether the aircraft is unclamped.
        wing_only: Whether only the wing is modelled.
        recover_accelerations: Whether accelerations are recovered in the linear model.

    Returns:
        str: Unique descriptive case name.
    """
    model_output   = 'ROM' if use_rom     else 'FOM'
    rbm_conditions = 'free_flight' if free_flight else 'clamped'
    name = 'superflexop_cruise_linear_{}_{}'.format(model_output, rbm_conditions)
    if wing_only:
        name += '_wing_only'
    if recover_accelerations:
        name += '_acc'
    return name


def run_linearization(
    use_rom: bool = True,
    aircraft_config: Optional[AircraftConfig] = None,
    run_config: Optional[RunConfig] = None,
    rom_settings: Optional[RomSettings] = None,
):
    """Run a SuperFLEXOP linearisation simulation.

    Args:
        use_rom: Produce a reduced-order model (ROM) in addition to the FOM.
        aircraft_config: Full aircraft configuration. Defaults to _AIRCRAFT_CONFIG.
            Use dataclasses.replace(_AIRCRAFT_CONFIG, wing_only=True) for partial overrides.
        run_config: Full run configuration. Defaults to _RUN_CONFIG.
        rom_settings: ROM settings. Defaults to _ROM_SETTINGS. Ignored when use_rom=False.

    Returns:
        Configured and executed FLEXOP model.
    """
    ac = aircraft_config or _AIRCRAFT_CONFIG
    rc = run_config      or _RUN_CONFIG
    rs = rom_settings    or _ROM_SETTINGS

    # Non-lifting body interactions: derive from lifting_only
    ac = dataclasses.replace(ac, nonlifting_interactions=not ac.lifting_only)

    flow = _build_flow(
        use_trim=True,
        restart=rc.restart_case,
    )

    case_name = _build_case_name(
        use_rom=use_rom,
        free_flight=rc.free_flight,
        wing_only=ac.wing_only,
        recover_accelerations=rc.recover_accelerations,
    )

    model = generate_flexop_case(
        U_INF, RHO, flow, _TRIM_VALUES, case_name,
        cases_route=_CASES_ROUTE,
        aircraft_config=ac,
        run_config=rc,
        rom_settings=rs if use_rom else RomSettings(use=False),
    )

    if not rc.restart_case:
        model.run()
    else:
        assert rc.restart_pickle_file is not None, \
            "restart_pickle_file must be set in run_config when restart_case=True"
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../05_Utils'))
        import restart_simulation_from_pickle as restart
        restart.restart_simulation_from_pickle(
            _CASES_ROUTE, case_name, rc.restart_pickle_file
        )

    return model


if __name__ == '__main__':
    run_linearization()
