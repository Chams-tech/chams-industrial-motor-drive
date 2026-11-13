# Motor Specifications — Reference PMSM

Status: **Phase 1 deliverable (v1.0)** · Simulation design targets, not a commercial motor datasheet.

## 1. Design basis

| Item                | Value                                        |
| ------------------- | -------------------------------------------- |
| Intended domain     | Industrial robotics and automation           |
| Target power class  | Below 500 W                                  |
| Simulation DC bus   | 48 V                                         |
| Motor technology    | Three-phase permanent-magnet synchronous machine (PMSM) |
| Rotor configuration | Surface-mounted magnets (SPM, non-salient)   |
| Number of phases    | 3                                            |
| Control technique   | Field-Oriented Control (FOC)                 |
| Position feedback   | Incremental encoder                          |
| Future controller   | STM32                                        |

The machine is **non-salient** (`Ld ≈ Lq`), so torque is produced only by the magnet flux linkage — no reluctance torque. (IPMSM saliency is a possible later extension; see `references.md`.)

## 2. Nameplate (simulation targets)

| Parameter                  | Symbol  | Value                     | Unit        |
| -------------------------- | ------- | ------------------------- | ----------- |
| Rated mechanical power     | P_N     | 400                       | W           |
| Rated speed                | n_N     | 3000                      | rpm         |
| Rated shaft torque         | T_N     | 1.27                      | N·m         |
| Rated phase current (RMS)  | I_N     | 8                         | A           |
| Continuous DC bus voltage  | V_dc    | 48 (range 40–56)          | V           |
| Pole pairs                 | p       | 4 (8 poles)               | –           |
| Continuous speed range     | –       | 0–3000                    | rpm         |
| Extended speed (field weakening) | – | 3000–4500               | rpm         |

*Check:* `T_N = P_N / ω_N = 400 / 314.16 = 1.27 N·m` ✓

## 3. Electrical parameters

| Parameter                       | Symbol   | Value                      | Unit     |
| ------------------------------- | -------- | -------------------------- | -------- |
| Phase resistance (20 °C)        | R_s      | 0.15                       | Ω        |
| Phase resistance (hot, 100 °C)  | R_s,hot  | 0.19                       | Ω        |
| d-axis inductance               | L_d      | 0.6                        | mH       |
| q-axis inductance               | L_q      | 0.6                        | mH       |
| Magnet flux linkage (peak amp.) | λ_pm     | 0.019                      | Wb       |
| Torque constant                 | K_t      | 0.161 (per A RMS)          | N·m/A    |
| Back-EMF constant at 3000 rpm   | K_e      | 16.9 (RMS, line-neutral)   | V        |
| Electrical time constant        | τ_e      | L/R = 4                    | ms       |

Notes:

- Convention: **peak amplitude** for dq quantities (see `motor-mathematical-model.md`).
- `K_t = 3/2 · √2 · p · λ_pm = 2.121 · 4 · 0.019 ≈ 0.161 N·m/A_rms`
- `K_e = λ_pm · ω_e / √2 = 0.019 · (4 · 314.16) / √2 ≈ 16.9 V_rms` at 3000 rpm
- Linear SVPWM limit (line-neutral fundamental RMS) at 48 V: `V_dc/√6 ≈ 19.6 V` → base-speed/modulation ceiling ≈ **3480 rpm**, so 3000 rpm rated leaves modulation margin.
- Inductance is sized for a current-loop pole at `R/L ≈ 250 rad/s (~40 Hz)`, allowing a PI current loop crossover of a few hundred Hz with a 10–20 kHz update rate.

## 4. Overload capability

| Parameter                    | Value                                  |
| ---------------------------- | -------------------------------------- |
| Peak phase current (RMS)     | 24 A (3 × rated), max 2 s              |
| Peak torque                  | ≈ 3.8 N·m (3 × rated)                  |
| Peak-to-continuous ratio     | 3:1                                    |
| Duty after overload          | Rest until thermal model permits       |

## 5. Mechanical parameters

| Parameter                         | Symbol | Value                   | Unit        |
| --------------------------------- | ------ | ----------------------- | ----------- |
| Rotor inertia (incl. load coupling) | J    | 2.5 × 10⁻⁵              | kg·m²      |
| Viscous friction coefficient      | B      | 1.0 × 10⁻⁴              | N·m·s/rad  |
| Coulomb friction torque           | T_c    | 0.02                    | N·m        |
| Max mechanical speed (continuous) | n_max  | 4500                    | rpm        |
| Mechanical time constant (voltage step, no load) | τ_m | J·R_s / (K_t·K_e) ≈ 0.43 | ms |
| Current-limited accel 0→3000 rpm (3× torque) | t_acc | ≈ 2.1 | ms |

*Checks:* `τ_m = 2.5e-5 · 0.15 / (0.161 · 0.0538) ≈ 0.43 ms` (K_e in V·s/rad = 16.9/314.16);
`t_acc = ω_N · J / T_peak = 314.16 · 2.5e-5 / 3.8 ≈ 2.1 ms` — small servos are this fast; the
100 ms settling target in `system-requirements.md` includes load inertia and controller tuning margins.

## 6. Thermal and protection limits

| Parameter                     | Value                  |
| ----------------------------- | ---------------------- |
| Motor continuous winding temp | ≤ 120 °C               |
| Thermal sensor                | NTC in winding (sim: lumped thermal model) |
| Inverter overcurrent trip     | 26 A (phase RMS peak detect) |
| Bus undervoltage trip         | 36 V                   |
| Bus overvoltage trip          | 58 V                   |

## 7. Sensing and PWM

| Parameter                       | Value                                        |
| ------------------------------- | -------------------------------------------- |
| PWM frequency                   | 20 kHz, center-aligned                       |
| Modulation                     | SVPWM (sinusoidal FOC + space-vector)        |
| Dead time                       | 0.8 µs (hardware; optional in early sim)     |
| Encoder                         | 1024 lines/rev → 4096 counts/rev (×4 decode) |
| Electrical angle resolution     | 4096 / 4 = 1024 counts per electrical rev    |
| Speed measurement               | Differentiation + filter (or M/T method)     |
| Current sensing                 | Three-shunt (default) — revisit in Phase 2   |
| Power stage                     | 60 V MOSFET bridge, ≥ 20 A continuous        |

## 8. Simulation parameter block (for Phase 2)

```python
# Reference PMSM — CHAMS industrial motor drive (Phase 1, v1.0)
MOTOR = dict(
    p=4,              # pole pairs
    V_dc=48.0,        # V
    R_s=0.15,         # ohm  (20 degC)
    L_d=0.6e-3,       # H
    L_q=0.6e-3,       # H
    lambda_pm=0.019,  # Wb   (peak amplitude convention)
    J=2.5e-5,         # kg.m^2
    B=1.0e-4,         # N.m.s/rad
    T_c=0.02,         # N.m  (Coulomb)
    I_rms_cont=8.0,   # A
    I_rms_peak=24.0,  # A  (2 s)
    n_rated=3000.0,   # rpm
    n_max=4500.0,     # rpm
)
```

## 9. Open items

- [ ] Confirm current-sensing topology (three-shunt vs single-shunt) — Phase 2
- [ ] Select exact STM32 part and ADC/PWM resources — Phase 3
- [ ] Validate parameter set against a real 400 W 48 V servo datasheet when available
- [ ] Add IPMSM saliency (L_d ≠ L_q) as an optional study case
