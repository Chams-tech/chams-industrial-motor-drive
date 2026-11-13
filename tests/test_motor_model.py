"""Unit tests for the SPMSM mathematical model."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation import motor_model as mm
from simulation.parameters import get_motor_parameters


def test_electrical_speed():
    assert mm.calculate_electrical_speed(100.0, 4) == 400.0


def test_torque_spmsm():
    p = get_motor_parameters()
    # iq = 11.31 A peak (rated 8 A RMS) -> T = 1.5 * 4 * 0.019 * 11.31
    t = mm.calculate_electromagnetic_torque(0.0, 11.31, p)
    assert abs(t - 1.5 * 4 * 0.019 * 11.31) < 1e-9
    # id has no effect on an SPSM
    assert mm.calculate_electromagnetic_torque(5.0, 11.31, p) == t


def test_current_derivatives_steady_state():
    """At vd = R*i, vq = we*(L*i + lambda) the derivatives vanish."""
    p = get_motor_parameters()
    id_c, iq_c = 0.0, 5.0
    we = 4 * 100.0
    vd = p["R_s"] * id_c - we * p["L_q"] * iq_c
    vq = p["R_s"] * iq_c + we * (p["L_d"] * id_c + p["lambda_pm"])
    did, diq = mm.calculate_current_derivatives(vd, vq, id_c, iq_c, we, p)
    assert abs(did) < 1e-9
    assert abs(diq) < 1e-9


def test_mechanical_acceleration():
    p = get_motor_parameters()
    # Pure electromagnetic torque at standstill, no load
    acc = mm.calculate_mechanical_acceleration(1.0, 0.0, 0.0, p)
    # sign(0) = +1 -> Coulomb friction subtracts
    assert acc == (1.0 - p["T_c"]) / p["J"]
    # Load torque reduces acceleration
    acc2 = mm.calculate_mechanical_acceleration(1.0, 0.5, 0.0, p)
    assert acc2 < acc


def test_rk4_constant_speed():
    """With steady-state voltages and a friction-balanced torque the speed
    stays constant."""
    p = get_motor_parameters()
    # Friction at 100 rad/s: B*w + T_c = 0.01 + 0.02 = 0.03 N.m
    # -> iq needed: 0.03 / (1.5*p*lambda) = 0.03 / 0.114
    iq_ss = 0.03 / (1.5 * p["p"] * p["lambda_pm"])
    state = {"id": 0.0, "iq": iq_ss, "speed": 100.0, "theta": 0.0}
    we = p["p"] * state["speed"]
    vd = -we * p["L_q"] * iq_ss
    vq = p["R_s"] * iq_ss + we * p["lambda_pm"]
    dt = 50e-6
    for _ in range(2000):  # 0.1 s
        state = mm.update_motor_state(state, vd, vq, 0.0, dt, p)
    assert abs(state["speed"] - 100.0) < 5.0
    assert abs(state["id"]) < 0.2
    assert abs(state["iq"] - iq_ss) < 0.2


def test_startup_acceleration():
    """Voltage commands that hold i_d=0, i_q=10 A accelerate the rotor."""
    p = get_motor_parameters()
    state = {"id": 0.0, "iq": 0.0, "speed": 0.0, "theta": 0.0}
    dt = 50e-6
    iq_target = 10.0
    for _ in range(400):  # 20 ms
        we = p["p"] * state["speed"]
        # Decoupled commands driving id -> 0 and iq -> 10 A
        vd = -we * p["L_q"] * state["iq"]
        vq = p["R_s"] * iq_target + we * p["lambda_pm"]
        state = mm.update_motor_state(state, vd, vq, 0.0, dt, p)
    assert state["speed"] > 100.0          # strong acceleration
    assert abs(state["id"]) < 0.5          # d-axis regulated near 0
    assert state["iq"] > 8.0               # q-axis tracks its target


def test_load_torque_slows_motor():
    p = get_motor_parameters()
    base = {"id": 0.0, "iq": 5.0, "speed": 100.0, "theta": 0.0}
    we = p["p"] * 100.0
    vd = -we * p["L_q"] * 5.0
    vq = p["R_s"] * 5.0 + we * p["lambda_pm"]
    dt = 50e-6
    s_free = dict(base)
    s_loaded = dict(base)
    for _ in range(2000):
        s_free = mm.update_motor_state(s_free, vd, vq, 0.0, dt, p)
        s_loaded = mm.update_motor_state(s_loaded, vd, vq, 0.5, dt, p)
    assert s_loaded["speed"] < s_free["speed"]
