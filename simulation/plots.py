"""Plotting utilities for the CHAMS PMSM FOC simulation.

All figures are written to the ``results/`` directory as PNG.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"


def _to_rpm(w):
    return np.asarray(w) * 30.0 / np.pi


def _save(fig, name: str, results_dir: Path):
    results_dir.mkdir(parents=True, exist_ok=True)
    path = results_dir / name
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_speed_response(results: dict, results_dir: Path = RESULTS_DIR) -> Path:
    t = np.asarray(results["time"])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, _to_rpm(results["speed"]), label="Actual speed", lw=1.4)
    if "speed_ref" in results:
        ax.plot(t, _to_rpm(results["speed_ref"]), "--", label="Reference", lw=1.2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [RPM]")
    ax.set_title("Speed response")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return _save(fig, "speed_response.png", results_dir)


def plot_current_response(results: dict, results_dir: Path = RESULTS_DIR) -> Path:
    t = np.asarray(results["time"])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, results["id"], label="i_d", lw=1.2)
    ax.plot(t, results["iq"], label="i_q", lw=1.2)
    if "iq_ref" in results:
        ax.plot(t, results["iq_ref"], "--", label="i_q ref", lw=1.0)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Current [A peak]")
    ax.set_title("dq currents")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return _save(fig, "current_response.png", results_dir)


def plot_torque_response(results: dict, results_dir: Path = RESULTS_DIR) -> Path:
    t = np.asarray(results["time"])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, results["torque"], label="Electromagnetic", lw=1.3)
    ax.plot(t, results["load_torque"], "--", label="Load", lw=1.2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Torque [N.m]")
    ax.set_title("Torque response")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return _save(fig, "torque_response.png", results_dir)


def plot_three_phase_currents(results: dict, results_dir: Path = RESULTS_DIR) -> Path:
    """Reconstruct i_a, i_b, i_c from the logged dq currents and angle."""
    t = np.asarray(results["time"])
    # Rebuild abc from logged id, iq requires the electrical angle;
    # approximate display reconstruction using Park inverse with theta
    # integrated from speed is not stored — so plot dq-derived envelope
    # instead when abc is unavailable.
    fig, ax = plt.subplots(figsize=(9, 4.5))
    if all(k in results for k in ("ia", "ib", "ic")):
        ax.plot(t, results["ia"], label="i_a", lw=1.0)
        ax.plot(t, results["ib"], label="i_b", lw=1.0)
        ax.plot(t, results["ic"], label="i_c", lw=1.0)
    else:
        iq = np.asarray(results["iq"])
        idc = np.asarray(results["id"])
        amp = np.hypot(idc, iq)
        ax.plot(t, amp, label="|i_dq| envelope", lw=1.0)
        ax.set_ylabel("Current [A peak]")
        ax.set_title("Three-phase current magnitude envelope "
                     "(phase waveforms in current_response)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        return _save(fig, "three_phase_currents.png", results_dir)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Current [A peak]")
    ax.set_title("Three-phase currents")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return _save(fig, "three_phase_currents.png", results_dir)


def plot_voltage_response(results: dict, results_dir: Path = RESULTS_DIR) -> Path:
    t = np.asarray(results["time"])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, results["vd"], label="v_d", lw=1.2)
    ax.plot(t, results["vq"], label="v_q", lw=1.2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Voltage [V]")
    ax.set_title("dq voltage commands")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return _save(fig, "voltage_response.png", results_dir)


def plot_duty_cycles(results: dict, results_dir: Path = RESULTS_DIR) -> Path:
    t = np.asarray(results["time"])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, results["duty_a"], label="d_a", lw=0.9)
    ax.plot(t, results["duty_b"], label="d_b", lw=0.9)
    ax.plot(t, results["duty_c"], label="d_c", lw=0.9)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Duty cycle [-]")
    ax.set_ylim(0.0, 1.0)
    ax.set_title("SVPWM duty cycles")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return _save(fig, "duty_cycles.png", results_dir)


def plot_all_results(results: dict, results_dir: Path = RESULTS_DIR) -> list:
    """Generate and save every standard plot; returns the list of paths."""
    return [
        plot_speed_response(results, results_dir),
        plot_current_response(results, results_dir),
        plot_torque_response(results, results_dir),
        plot_three_phase_currents(results, results_dir),
        plot_voltage_response(results, results_dir),
        plot_duty_cycles(results, results_dir),
    ]
