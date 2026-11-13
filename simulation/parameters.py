"""Centralized configuration for the CHAMS PMSM FOC simulation.

Conventions
-----------
- SI units internally: rad/s, rad, A, V, N.m, H, Ohm, kg.m^2.
- dq currents/voltages use the **peak amplitude** convention
  (Phase 1 motor-specifications.md): rated 8 A RMS == 8*sqrt(2) ~= 11.31 A peak.
- Mechanical speed in rad/s; RPM values are converted by the caller.
- Electrical angle theta is in radians (electrical), theta = p * theta_m.
"""

from __future__ import annotations

import math

# ----------------------------------------------------------------------------
# Reference motor 
# ----------------------------------------------------------------------------

def get_motor_parameters() -> dict:
    """Return the electrical and mechanical parameters of the reference PMSM."""
    return {
        # Electrical
        "V_dc": 48.0,               # DC bus voltage [V]
        "R_s": 0.15,                # Stator resistance, 20 degC [Ohm]
        "L_d": 0.6e-3,              # d-axis inductance [H]
        "L_q": 0.6e-3,              # q-axis inductance [H]
        "lambda_pm": 0.019,         # Magnet flux linkage [Wb] (peak amplitude)
        "p": 4,                     # Pole pairs [-]
        # Mechanical
        "J": 2.5e-5,                # Rotor inertia [kg.m^2]
        "B": 1.0e-4,                # Viscous friction [N.m.s/rad]
        "T_c": 0.02,                # Coulomb friction [N.m]
        # Ratings (peak-amplitude current convention)
        "n_rated_rpm": 3000.0,      # Rated speed [rpm]
        "I_rms_rated": 8.0,         # Rated phase current [A RMS]
        "I_peak_rated": 8.0 * math.sqrt(2.0),   # Rated dq current [A peak] ~= 11.31
        "I_peak_max": 24.0,         # Peak dq current limit [A peak] (short duration)
        # PWM
        "f_pwm": 20_000.0,          # PWM frequency [Hz]
    }


# ----------------------------------------------------------------------------
# Simulation configuration
# ----------------------------------------------------------------------------

def get_simulation_parameters() -> dict:
    """Return simulation timing and controller configuration.

    Timing
    ------
    - Current control / PWM : 20 kHz  -> dt = 50 us
    - Speed control         : 1 kHz   -> every 20 current-loop steps
    """
    dt = 1.0 / 20_000.0            # 50 us, matches the PWM/current-loop rate
    speed_divider = 20             # speed loop runs at 1 kHz
    return {
        "dt": dt,                          # simulation / current-loop time step [s]
        "f_current": 1.0 / dt,             # 20000 Hz
        "speed_divider": speed_divider,    # speed loop decimation factor
        "f_speed": 1.0 / (dt * speed_divider),  # 1000 Hz
        "duration": 1.0,                   # default simulation duration [s]
        # Controller gains (see docs/PHASE_2.md design notes)
        # Current loop: Kp = L*w_c, Ki = R*w_c with w_c = 1000 rad/s
        "current_kp": 0.6e-3 * 1000.0,     # 0.6
        "current_ki": 0.15 * 1000.0,       # 150.0
        # Speed loop: type-II design, w_c = 100 rad/s, PM ~ 60 deg
        "speed_kp": 0.027,
        "speed_ki": 1.55,
    }
