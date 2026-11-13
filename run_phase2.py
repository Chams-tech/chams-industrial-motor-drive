"""Phase 2 scenario runner — final simulation tests (docs/PHASE_2.md §4).

Usage:
    python run_phase2.py            # run all scenarios, print metrics, save plots
    python run_phase2.py startup    # run a single scenario
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

from simulation.plots import plot_all_results
from simulation.simulation import compute_speed_metrics, run_simulation

RPM = 60.0 / (2.0 * math.pi)   # rad/s -> RPM conversion factor
RESULTS = Path(__file__).resolve().parent / "results"


def rpm(w: float) -> float:
    return w * RPM


def w(r: float) -> float:
    """RPM -> rad/s."""
    return r / RPM


def scenario_startup(duration: float = 0.4) -> dict:
    """0 -> 1000 RPM, no load: smooth acceleration."""
    return run_simulation(w(1000.0), 0.0, duration)


def scenario_speed_tracking(duration: float = 0.6) -> dict:
    """1000 -> 2000 RPM step at t = 0.2 s."""
    ref = lambda t: w(1000.0) if t < 0.2 else w(2000.0)
    return run_simulation(ref, 0.0, duration)


def scenario_rated_speed(duration: float = 0.6) -> dict:
    """0 -> 3000 RPM: stable rated-speed operation."""
    return run_simulation(w(3000.0), 0.0, duration)


def scenario_load_disturbance(duration: float = 1.0) -> dict:
    """0 -> 3000 RPM; 0.5 N.m load step at t = 0.5 s."""
    load = lambda t: 0.0 if t < 0.5 else 0.5
    return run_simulation(w(3000.0), load, duration)


def scenario_deceleration(duration: float = 0.6) -> dict:
    """3000 -> 1000 RPM step at t = 0.2 s: controlled deceleration."""
    ref = lambda t: w(3000.0) if t < 0.2 else w(1000.0)
    return run_simulation(ref, 0.0, duration)


def scenario_current_limit(duration: float = 0.4) -> dict:
    """Aggressive start under load: current must stay within limits."""
    return run_simulation(w(3000.0), 1.0, duration)


def scenario_voltage_limit(duration: float = 0.8) -> dict:
    """Command 6000 RPM: voltage must respect the DC bus limits."""
    return run_simulation(w(6000.0), 0.0, duration)


SCENARIOS = {
    "startup": scenario_startup,
    "tracking": scenario_speed_tracking,
    "rated": scenario_rated_speed,
    "load": scenario_load_disturbance,
    "decel": scenario_deceleration,
    "current_limit": scenario_current_limit,
    "voltage_limit": scenario_voltage_limit,
}


def check_results(name: str, res: dict, motor_peak_limit: float = 24.0,
                  v_limit: float = 48.0 / math.sqrt(3.0)) -> list:
    """Return a list of PASS/FAIL strings for the hard constraints."""
    lines = []
    duties = res["duty_a"] + res["duty_b"] + res["duty_c"]
    vdq = [math.hypot(a, b) for a, b in zip(res["vd"], res["vq"])]
    iq_max = max(abs(v) for v in res["iq"])
    id_max = max(abs(v) for v in res["id"])

    ok_duty = min(duties) >= -1e-9 and max(duties) <= 1.0 + 1e-9
    ok_v = max(vdq) <= v_limit + 1e-6
    ok_i = iq_max <= motor_peak_limit + 1e-6 and id_max <= motor_peak_limit + 1e-6

    lines.append(f"[{'PASS' if ok_duty else 'FAIL'}] {name}: duties in [0,1] "
                 f"(min={min(duties):.3f}, max={max(duties):.3f})")
    lines.append(f"[{'PASS' if ok_v else 'FAIL'}] {name}: |v_dq| <= {v_limit:.2f} V "
                 f"(max={max(vdq):.2f} V)")
    lines.append(f"[{'PASS' if ok_i else 'FAIL'}] {name}: |i_dq| <= {motor_peak_limit:.1f} A "
                 f"(max iq={iq_max:.1f} A, max id={id_max:.1f} A)")
    return lines


def report(name: str, res: dict) -> str:
    """One-line summary of the speed performance of a scenario."""
    final_ref = res["speed_ref"][-1]
    m = compute_speed_metrics(res, final_ref)
    return (f"{name:14s} ref={rpm(final_ref):6.0f} RPM  "
            f"rise={m['rise_time'] * 1e3:6.1f} ms  "
            f"settle={m['settling_time'] * 1e3:7.1f} ms  "
            f"ss_err={rpm(m['steady_state_error']):5.2f} RPM  "
            f"overshoot={rpm(m['overshoot']):5.1f} RPM")


def main() -> int:
    names = sys.argv[1:] or list(SCENARIOS)
    all_lines = []
    print("=" * 100)
    for name in names:
        res = SCENARIOS[name]()
        print(report(name, res))
        all_lines += check_results(name, res)
        if len(sys.argv) == 1:  # full run: save plots for every scenario
            plot_all_results(res, RESULTS / name)
    print("-" * 100)
    for line in all_lines:
        print(line)
    failed = sum(1 for ln in all_lines if "FAIL" in ln)
    print("-" * 100)
    print(f"{len(all_lines) - failed}/{len(all_lines)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
