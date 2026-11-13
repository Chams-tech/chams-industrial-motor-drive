# Motor Mathematical Model

Status: **Phase 1 — outline (equations to be completed next)**

Target machine: three-phase SPM PMSM, `L_d = L_q = L_s`, per `motor-specifications.md`.
Convention: **peak amplitude** for dq/abc quantities; `p` = pole pairs; `ω_e = p · ω_m`.

## 1. abc stationary frame

```
v_abc = R_s · i_abc + dλ_abc/dt
λ_abc = L_s · i_abc + λ_pm · [cos θ_e, cos(θ_e − 2π/3), cos(θ_e + 2π/3)]ᵀ
T_e   = (3/2) · p · ( i_a·dλ_a/dθ_e + … )   → see dq form below
```

## 2. Clarke and Park transforms

```
Clarke  (abc → αβ):  x_αβ = (2/3) · [1, −1/2, −1/2; 0, √3/2, −√3/2] · x_abc
Park    (αβ → dq):   x_dq = R(θ_e) · x_αβ ,  R(θ_e) = [cos θ_e, sin θ_e; −sin θ_e, cos θ_e]
Inverse as usual; zero-sequence neglected (isolated neutral).
```

## 3. dq voltage and torque equations (SPM)

```
v_d = R_s·i_d + L_d·di_d/dt − ω_e·L_q·i_q
v_q = R_s·i_q + L_q·di_q/dt + ω_e·(L_d·i_d + λ_pm)

T_e = (3/2)·p·[ λ_pm·i_q + (L_d − L_q)·i_d·i_q ]
    = (3/2)·p·λ_pm·i_q                 (SPM: L_d = L_q, no reluctance torque)
```

## 4. Mechanical equation

```
J·dω_m/dt = T_e − T_L − B·ω_m − T_c·sign(ω_m)
θ_e = p·θ_m + θ_offset
```

## 5. Inverter model (ideal, for Phase 2)

```
v_dc bus, two-level 3φ VSI, SVPWM:
  linear range: |v_αβ| ≤ V_dc / √3   (phase-neutral peak)
duty cycles → switching states; optional dead-time and voltage-limit clamp.
```

## 6. Control-oriented forms (preview)

- Current plant: first-order-ish with pole at `R_s/L_s`, cross-coupling terms `ω_e·L·i` as disturbances.
- Cascade loops: PI on `i_d, i_q` (inner, 10–20 kHz) → PI on speed (outer, ~1 kHz).
- Reference strategy: `i_d = 0` below base speed; field weakening `i_d < 0` above ~3480 rpm.

## 7. Next steps

- [ ] Complete derivations with sign/convention notes and simulation-ready state-space form
- [ ] Steady-state charts: torque–speed envelope (constant torque + field weakening)
- [ ] Parameterize the model from `motor-specifications.md` §8 and export to `simulation/`
