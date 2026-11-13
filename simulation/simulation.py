"""Simulation engine: couples the SPMSM model with the FOC controller.

Timing
------
- Simulation / current loop / PWM : dt = 50 us (20 kHz)
- Speed loop                      : every 20 ticks (1 kHz)

Signal flow per tick
--------------------
FOC(currents, theta, speed_ref, speed) -> duties
    -> average inverter: v_xN = V_dc * (d_x - 1/3),  sum(v_xN) = 0
    -> Clarke: v_alpha-beta
    -> inverse Park: v_d, v_q  (plant inputs)
    -> motor RK4 step
    -> ideal sensors: phase currents via inverse Park/Clarke for the next tick
"""

from __future__ import annotations

import math

from .foc_controller import FOCController
from .motor_model import update_motor_state
from .parameters import get_motor_parameters, get_simulation_parameters
from .transformations import (clarke_transform, inverse_clarke_transform,
                              inverse_park_transform, park_transform)


def _inverter_voltages(duty: tuple[float, float, float],
                       v_dc: float) -> tuple[float, float, float]:
    """Average phase-to-neutral voltages of a two-level VSI (sum = 0).

    v_xN = V_dc * (d_x - mean(d)) — exact for centered PWM with any
    zero-sequence injection; guarantees v_a + v_b + v_c = 0.
    """
    da, db, dc = duty
    mean_d = (da + db + dc) / 3.0
    return (v_dc * (da - mean_d),
            v_dc * (db - mean_d),
            v_dc * (dc - mean_d))


def run_simulation(speed_reference, load_torque, simulation_duration,
                   parameters=None) -> dict:
    """Run the FOC simulation.

    Parameters
    ----------
    speed_reference : float or callable(t) -> rad/s
    load_torque     : float or callable(t) -> N.m
    simulation_duration : float [s]
    parameters : optional dict with keys "motor" and "sim" (defaults applied)

    Returns
    -------
    results : dict of lists (time, speed, id, iq, torque, load_torque,
              vd, vq, duty_a, duty_b, duty_c, iq_ref, v_alpha, v_beta).
    Speeds in rad/s, currents in A (peak dq), voltages in V, torque in N.m.
    """
    parameters = parameters or {}
    motor = parameters.get("motor") or get_motor_parameters()
    sim = parameters.get("sim") or get_simulation_parameters()

    dt = sim["dt"]
    n_steps = int(round(simulation_duration / dt))

    ref_fn = speed_reference if callable(speed_reference) else (
        lambda _t, r=speed_reference: r)
    load_fn = load_torque if callable(load_torque) else (
        lambda _t, tl=load_torque: tl)

    state = {"id": 0.0, "iq": 0.0, "speed": 0.0, "theta": 0.0}
    controller = FOCController(motor, sim)

    results = {k: [] for k in (
        "time", "speed", "speed_ref", "id", "iq", "iq_ref", "torque",
        "load_torque", "vd", "vq", "duty_a", "duty_b", "duty_c",
        "ia", "ib", "ic", "v_alpha", "v_beta")}

    for k in range(n_steps):
        t = k * dt
        measured_speed = state["speed"]
        electrical_angle = state["theta"]
        load = float(load_fn(t))
        speed_ref = float(ref_fn(t))

        # Ideal current sensors: reconstruct abc from the motor dq state
        valpha, vbeta = inverse_park_transform(state["id"], state["iq"],
                                               electrical_angle)
        ia, ib, ic = inverse_clarke_transform(valpha, vbeta)

        # FOC tick
        duty = controller.update((ia, ib, ic), electrical_angle,
                                 speed_ref, measured_speed, dt)

        # Average inverter -> plant voltages
        van, vbn, vcn = _inverter_voltages(duty, motor["V_dc"])
        v_alpha, v_beta = clarke_transform(van, vbn, vcn)
        vd, vq = park_transform(v_alpha, v_beta, electrical_angle)

        # Motor integration (RK4)
        state = update_motor_state(state, vd, vq, load, dt, motor)

        # Logging
        torque = 1.5 * motor["p"] * motor["lambda_pm"] * controller.iq_meas
        results["time"].append(t)
        results["speed"].append(state["speed"])
        results["speed_ref"].append(speed_ref)
        results["id"].append(controller.id_meas)
        results["iq"].append(controller.iq_meas)
        results["iq_ref"].append(controller._iq_ref)
        results["torque"].append(torque)
        results["load_torque"].append(load)
        results["vd"].append(controller.v_d)
        results["vq"].append(controller.v_q)
        results["duty_a"].append(duty[0])
        results["duty_b"].append(duty[1])
        results["duty_c"].append(duty[2])
        results["ia"].append(ia)
        results["ib"].append(ib)
        results["ic"].append(ic)
        results["v_alpha"].append(v_alpha)
        results["v_beta"].append(v_beta)

    return results


# ---------------------------------------------------------------------------- #
# Performance metrics
# ---------------------------------------------------------------------------- #

def compute_speed_metrics(results: dict, ref_final: float,
                          settle_band: float = 0.02) -> dict:
    """Step-response metrics of the **last** speed reference change.

    The analysis window starts at the last reference change (or t=0 if the
    reference is constant). Returns rise time (10-90%), settling time
    (within ``settle_band`` of ref_final), steady-state error and overshoot
    (positive for upward steps, reported as 0 for downward steps where the
    metric is an undershoot).
    """
    import numpy as np

    t = np.asarray(results["time"])
    w = np.asarray(results["speed"])
    ref = float(ref_final)

    # Window: after the last reference change
    if "speed_ref" in results:
        ref_arr = np.asarray(results["speed_ref"])
        changes = np.where(np.diff(ref_arr) != 0.0)[0]
        start = int(changes[-1] + 1) if len(changes) else 0
    else:
        start = 0
    t = t[start:] - (t[start] if start < len(t) else 0.0)
    w = w[start:]

    if len(w) < 2:
        return {"rise_time": float("nan"), "settling_time": float("nan"),
                "steady_state_error": float("nan"), "overshoot": 0.0,
                "w_ss": float(w[-1]) if len(w) else 0.0}

    # Steady-state: average over the last 5% of the window
    tail = w[int(0.95 * len(w)):]
    w_ss = float(np.mean(tail))
    ss_error = abs(ref - w_ss)

    if abs(ref) < 1e-9:
        return {"rise_time": 0.0, "settling_time": 0.0,
                "steady_state_error": ss_error, "overshoot": 0.0,
                "w_ss": w_ss}

    # Rise time 10% -> 90% (relative to the window start value)
    w0 = w[0]
    level_lo = w0 + 0.1 * (ref - w0)
    level_hi = w0 + 0.9 * (ref - w0)
    if ref >= w0:
        idx10 = int(np.argmax(w >= level_lo))
        idx90 = int(np.argmax(w >= level_hi))
    else:  # downward step
        idx10 = int(np.argmax(w <= level_lo))
        idx90 = int(np.argmax(w <= level_hi))
    rise_time = float(t[idx90] - t[idx10]) if idx90 > idx10 else float("nan")

    # Settling: last time the response leaves the band
    band = settle_band * abs(ref)
    outside = np.where(np.abs(w - ref) > band)[0]
    settling_time = float(t[outside[-1]] + dt_of(results)) if len(outside) else 0.0

    # Overshoot (upward) / undershoot magnitude (downward)
    if ref >= w0:
        overshoot = float(max(0.0, np.max(w) - ref))
    else:
        overshoot = float(max(0.0, ref - np.min(w)))

    return {"rise_time": rise_time, "settling_time": settling_time,
            "steady_state_error": ss_error, "overshoot": overshoot,
            "w_ss": w_ss}


def dt_of(results: dict) -> float:
    t = results["time"]
    return (t[1] - t[0]) if len(t) > 1 else 0.0
