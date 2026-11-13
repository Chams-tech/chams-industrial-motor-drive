# Phase 2 — PMSM Field-Oriented Control (FOC) Simulation

## 1. Phase objective

Develop a complete Python simulation of a three-phase Surface-Mounted PMSM (SPMSM) controlled
using Field-Oriented Control (FOC).

The simulation must demonstrate:

- Motor startup and acceleration
- Speed regulation
- Torque control
- Current regulation
- Three-phase voltage generation
- Rejection of external load disturbances
- Motor behaviour under different operating conditions

This phase is entirely simulation-based. No physical motor, inverter, STM32 or hardware is required.

### Development tools

- Python 3
- NumPy — mathematical calculations
- SciPy — numerical integration, if needed
- Matplotlib — simulation plots
- Pytest — unit testing

---

## 2. Project structure

```text
chams-industrial-motor-drive/
│
├── docs/
│   ├── project-overview.md
│   ├── motor-specifications.md
│   ├── motor-mathematical-model.md
│   └── PHASE_2.md
│
├── simulation/
│   ├── __init__.py
│   ├── parameters.py
│   ├── motor_model.py
│   ├── transformations.py
│   ├── pi_controller.py
│   ├── current_controller.py
│   ├── speed_controller.py
│   ├── svpwm.py
│   ├── foc_controller.py
│   ├── simulation.py
│   └── plots.py
│
├── tests/
│   ├── test_motor_model.py
│   ├── test_transformations.py
│   ├── test_pi_controller.py
│   └── test_svpwm.py
│
├── results/
│
├── requirements.txt
└── README.md
```

---

## 3. Step-by-step development

### Step 1 — Motor parameters and configuration (`simulation/parameters.py`)

Centralized configuration imported by every simulation module, using the Phase 1 reference motor:

| Parameter | Value |
|---|---:|
| DC bus voltage | 48 V |
| Stator resistance | 0.15 Ω |
| d-axis inductance | 0.6 mH |
| q-axis inductance | 0.6 mH |
| PM flux linkage | 0.019 Wb |
| Pole pairs | 4 |
| Rotor inertia | 2.5 × 10⁻⁵ kg·m² |
| Viscous friction | 1 × 10⁻⁴ N·m·s/rad |
| Coulomb friction | 0.02 Nm |
| Rated speed | 3000 RPM |
| Rated current | 8 A RMS |
| PWM frequency | 20 kHz |

Functions:

- `get_motor_parameters()` — electrical and mechanical parameters
- `get_simulation_parameters()` — simulation frequency, time step and duration

Validation: consistent units, henries for inductance, rad/s internally, RMS/peak conventions documented.

### Step 2 — PMSM mathematical model (`simulation/motor_model.py`)

Electrical equations:

    di_d/dt = (v_d − R_s·i_d + ω_e·L_q·i_q) / L_d
    di_q/dt = (v_q − R_s·i_q − ω_e·(L_d·i_d + λ_pm)) / L_q

Torque (SPMSM, L_d = L_q):

    T_e = (3/2)·p·λ_pm·i_q

Mechanical equation:

    J·dω_m/dt = T_e − T_L − B·ω_m − T_c·sign(ω_m)
    ω_e = p·ω_m

Functions: `calculate_electrical_speed`, `calculate_electromagnetic_torque`,
`calculate_current_derivatives`, `calculate_mechanical_acceleration`, `update_motor_state`
(RK4 integration).

Motor state: `{"id", "iq", "speed", "theta"}` — amps, rad/s, electrical radians.

### Step 3 — Clarke and Park transformations (`simulation/transformations.py`)

- Clarke (abc → αβ): `i_α = i_a`, `i_β = (i_a + 2·i_b)/√3`
- Park (αβ → dq): `i_d = i_α cos θ_e + i_β sin θ_e`, `i_q = −i_α sin θ_e + i_β cos θ_e`
- Inverse Park: `v_α = v_d cos θ_e − v_q sin θ_e`, `v_β = v_d sin θ_e + v_q cos θ_e`
- Inverse Clarke: `v_a = v_α`, `v_b = −v_α/2 + (√3/2)·v_β`, `v_c = −v_α/2 − (√3/2)·v_β`

Validation: reconstruction round-trips within numerical tolerance at multiple angles.

### Step 4 — PI controller (`simulation/pi_controller.py`)

    e = reference − measurement
    u = K_p·e + K_i·∫e dt

Class `PIController(kp, ki, output_min, output_max)` with `update(reference, measurement, dt)`
and `reset()`. Output saturation with anti-windup.

### Step 5 — Current controllers (`simulation/current_controller.py`)

Independent d/q PI loops with decoupling compensation:

    v_d* = PI_d(e_d) − ω_e·L_q·i_q
    v_q* = PI_q(e_q) + ω_e·(L_d·i_d + λ_pm)

Class `CurrentController(parameters, gains)` with `update(...)` and `reset()`.
Initial d-axis reference: `i_d* = 0`.

### Step 6 — SVPWM (`simulation/svpwm.py`)

Functions: `calculate_sector`, `calculate_svpwm_duties`, `limit_voltage_vector`.
Linear range: `√(v_α² + v_β²) ≤ V_dc/√3`. Duty cycles in [0, 1], PWM = 20 kHz.

### Step 7 — Speed controller (`simulation/speed_controller.py`)

    e_ω = ω_m* − ω_m
    i_q* = PI_ω(e_ω),  limited to the configured current limit
    i_d* = 0

Class `SpeedController(kp, ki, current_limit)` with anti-windup.

### Step 8 — Integrated FOC controller (`simulation/foc_controller.py`)

Sequence: read currents → Clarke → Park → current errors → d/q PI + decoupling →
inverse Park → SVPWM duties → inverter voltage commands.

Class `FOCController(motor_parameters, control_parameters)` with `update(...)` and `reset()`.

### Step 9 — Simulation engine (`simulation/simulation.py`)

| Component | Frequency |
|---|---:|
| PWM / current control | 20 kHz |
| Speed control | 1 kHz |
| Simulation time step | 50 µs |

`run_simulation(speed_reference, load_torque, simulation_duration, parameters)` records
`time, speed, id, iq, torque, load_torque, vd, vq, duty_a, duty_b, duty_c`.

Important: the motor plant receives the actual inverter voltage produced by the PWM
(duty cycles → average phase voltages → dq), not the raw PI voltage request.

### Step 10 — Visualization (`simulation/plots.py`)

Speed, currents, torque, three-phase currents, voltages, duty cycles — saved to `results/`.

---

## 4. Final simulation tests

| Test | Input | Expected result |
|---|---|---|
| Startup | 0 → 1000 RPM | Smooth acceleration |
| Speed tracking | 1000 → 2000 RPM | Speed follows reference |
| Rated speed | 3000 RPM | Stable rated-speed operation |
| Load disturbance | Increase load torque | Speed recovers |
| Speed reduction | 3000 → 1000 RPM | Controlled deceleration |
| Current limitation | High torque request | Current remains limited |
| Voltage limitation | High speed request | Voltage respects DC bus limits |

Performance measurements: rise time, settling time, steady-state speed error, overshoot,
current tracking error, torque response time — compared against Phase 1 targets.

---

## 5. Phase 2 completion criteria

- [x] PMSM mathematical model works
- [x] Clarke and Park transformations pass their tests
- [x] PI controller works correctly
- [x] d-axis and q-axis current controllers work
- [x] SVPWM generates valid duty cycles
- [x] Speed controller regulates motor speed
- [x] Complete FOC controller is integrated
- [x] Simulation engine runs successfully
- [x] Required graphs are generated
- [x] Load disturbance tests are completed
- [x] Current and voltage limits are respected
- [x] Results and test methodology are documented

---

## 5.1 Measured results (simulation, `python run_phase2.py`)

Test suite: **22/22 unit tests**, **21/21 scenario checks** pass.

| Scenario        | Reference            | Rise time | Settling time (2 %) | Steady-state error | Notes |
|-----------------|----------------------|----------:|--------------------:|-------------------:|-------|
| Startup         | 0 → 1000 RPM         | 8.9 ms    | 54.5 ms             | 0.00 RPM           | overshoot 188 RPM |
| Speed tracking  | 1000 → 2000 RPM      | 8.3 ms    | 49.3 ms             | 0.00 RPM           | overshoot 217 RPM |
| Rated speed     | 0 → 3000 RPM         | 8.6 ms    | 55.6 ms             | 0.00 RPM           | modulation ceiling reached |
| Load disturbance| 0.5 N.m step @ 0.5 s | —         | 46 ms after step    | 0.00 RPM           | speed recovers to 3000 RPM |
| Deceleration    | 3000 → 1000 RPM      | 8.3 ms    | 58.4 ms             | 0.00 RPM           | controlled braking |
| Current limit   | 3000 RPM @ 1.0 N.m   | 23.8 ms   | 39.0 ms             | 0.00 RPM           | max i_q = 12.1 A ≤ 24 A |
| Voltage limit   | 6000 RPM command     | —         | —                   | —                  | caps at ≈ 3472 RPM (V_dc/√3 ceiling ≈ 3480 RPM) |

Hard constraints verified in every scenario:

- duty cycles ∈ [0, 1]
- |v_dq| ≤ V_dc/√3 = 27.71 V
- |i_d|, |i_q| ≤ 24 A peak

Comparison with Phase 1 targets (`system-requirements.md`):

| Metric                        | Target        | Measured (startup)   |
|-------------------------------|---------------|----------------------|
| Speed settling (0 → rated)    | < 100 ms      | 55.6 ms ✓            |
| Steady-state speed accuracy   | ±0.5 %        | 0.00 RPM ✓           |
| Torque response (step to 50 %)| < 10 ms       | current loop < 1 ms (i_q tracks i_q* within one tick) ✓ |

**Notes**

- Overshoot ≈ 18–19 % on unloaded speed steps is inherent to the PI cascade with
  the current loop left unconstrained at light load; it remains within the
  Phase 1 envelope (no overshoot target was defined).
- The voltage-limit scenario intentionally commands 6000 RPM: the drive saturates
  at the linear-modulation ceiling (≈ 3472 RPM), validating the V_dc/√3 limit.
- Field weakening (`i_d < 0` above base speed) is not yet implemented — the
  drive clips at base speed. This is a documented Phase 2.5 extension.

---

## 6. Development rules

1. Implement one module at a time.
2. Test every function before integrating it.
3. Do not implement the entire FOC system in one Python file.
4. Keep motor parameters centralized.
5. Document equations and units.
6. Save simulation results and plots.
7. Commit each completed development step to GitHub.
8. Do not proceed to STM32 firmware until the simulation is validated.

---

## 7. Expected result

A working Python-based PMSM FOC simulation where the speed reference, load torque and motor
parameters can be changed, the simulation executed, and the motor response observed —
the validated foundation for Phase 3 (real-time FOC on STM32).
