"""
Shared cruise flight conditions and configuration dataclasses for the
SuperFLEXOP aircraft at 800 m altitude.

All simulation scripts import U_INF, RHO, and the dataclasses defined here,
then instantiate them with case-specific values.  The cruise baseline values
are the dataclass field defaults.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional, Union

U_INF = 45.0    # cruise speed [m/s]
RHO   = 1.1336  # air density at 800 m altitude [kg/m^3]


@dataclass
class TrimValues:
    """Initial trim state for a simulation case (no defaults: always case-specific)."""
    alpha:  float  # angle of attack [rad]
    delta:  float  # elevator deflection [rad]
    thrust: float  # engine thrust [N]


@dataclass
class AircraftConfig:
    """Aircraft model and aero/structural discretisation parameters."""
    sigma:                      float          = 0.3    # stiffness scaling (1 = FLEXOP, 0.3 = SuperFLEXOP)
    n_elem_multiplier:          int            = 2      # spanwise structural node multiplier
    n_elem_multiplier_fuselage: int            = 1      # longitudinal fuselage node multiplier
    num_chord_panels:           int            = 8      # chordwise aero lattice panels
    mstar:                      int            = 80     # streamwise wake panels
    lifting_only:               bool           = True   # ignore non-lifting bodies
    wing_only:                  bool           = False  # wing-only model (no tail, no fuselage)
    horseshoe:                  bool           = False  # use horseshoe wake
    variable_wake:              bool           = False  # variable wake discretisation
    use_polars:                 bool           = False  # apply polar corrections
    gravity:                    bool           = True   # include gravity
    nonlifting_interactions:    bool           = False  # include non-lifting body interactions from fuselage
    ailerons_type:              int            = 0      # aileron type flag (0 = standard, 1 = dynamic, 2 = controlled)
    num_radial_panels:          int            = 24     # fuselage radial panels
    delta_x_payload:            Optional[float] = None  # payload CG x-offset [m]
    dict_wake_shape:            Optional[dict]  = None  # variable wake shape parameters


@dataclass
class RunConfig:
    """Solver and execution parameters."""
    n_tstep:                         int           = 1      # number of time steps
    num_cores:                       int           = 2      # CPU cores for parallelisation
    free_flight:                     bool          = True   # free-flying vs clamped aircraft
    num_modes:                       int           = 20     # number of structural modes
    recover_accelerations:           bool          = False  # recover accelerations in linear model
    remove_gust_input_in_statespace: bool          = False  # remove gust from state-space inputs
    postprocessors_dynamic:          list          = field(default_factory=lambda: ['BeamLoads', 'SaveData'])
    n_load_steps:                    int           = 5      # static load steps
    dynamic_cs_input:                bool          = False  # use pre-defined CS deflections
    dynamic_cs_input_file:           Optional[str] = None   # path to CS deflection time series
    dict_predefined_cs_input_files:  Optional[dict] = None  # per-CS deflection file mapping
    restart_case:                    bool          = False  # restart from pickle
    restart_pickle_file:             Optional[str] = None   # path to restart pickle
    save_pickle_file:                bool          = False  # save pickle at end
    output_folder:                   str           = './output/'


@dataclass
class GustConfig:
    """Gust excitation parameters.

    Fields without defaults (gust_shape, gust_offset, gust_component) are
    always required.  Shape-specific fields default to sensible values but
    are only meaningful for the matching gust_shape.
    """
    gust_shape:     str                     # '1-cos' or 'time varying'
    gust_offset:    float                   # streamwise start offset [m]
    gust_component: Union[int, List[int]]   # velocity component: 0=x, 1=y, 2=z
    gust_length:    float         = 10.0    # gust wavelength [m]         (1-cos only)
    gust_intensity: float         = 0.1     # intensity as fraction of U_INF (1-cos only)
    file:           Optional[str] = None    # path to time-series file    (time-varying only)


@dataclass
class RomSettings:
    """Reduced-order model settings."""
    use:                 bool = False
    rom_method:          list = field(default_factory=lambda: ['Krylov'])
    rom_method_settings: dict = field(default_factory=dict)
