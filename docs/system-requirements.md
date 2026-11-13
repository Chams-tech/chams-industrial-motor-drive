# System Requirements

Functional and safety requirements for the CHAMS industrial motor drive.
Each requirement has a stable ID for traceability into simulation and firmware tests.

## 1. Motion control

| ID      | Requirement                                                                 | Priority |
| ------- | --------------------------------------------------------------------------- | -------- |
| F-001   | Start the motor smoothly from standstill without current limit violations   | Must     |
| F-002   | Stop the motor smoothly (controlled ramp-down), including to standstill      | Must     |
| F-003   | Maintain a requested reference speed within a defined steady-state band     | Must     |
| F-004   | Reject disturbances caused by changes in mechanical load (speed droop/recovery) | Must  |
| F-005   | Control electromagnetic torque directly (torque command mode)               | Must     |
| F-006   | Support both speed-control and torque-control modes with bumpless switching | Should   |

## 2. Monitoring and protection

| ID      | Requirement                                                                 | Priority |
| ------- | --------------------------------------------------------------------------- | -------- |
| M-001   | Continuously measure/estimate three-phase motor currents                   | Must     |
| M-002   | Continuously measure rotor position and speed from the encoder              | Must     |
| M-003   | Detect overcurrent and command an immediate safe stop                      | Must     |
| M-004   | Detect abnormal operating conditions (overvoltage, undervoltage, overheating, encoder loss) | Must |
| M-005   | Latch the fault state; require an explicit reset before restarting          | Must     |
| M-006   | Log/expose fault codes for diagnostics                                      | Should   |

## 3. Safety behavior

| ID      | Requirement                                                                 | Priority |
| ------- | --------------------------------------------------------------------------- | -------- |
| S-001   | On any critical fault, disable the PWM outputs before the next control tick | Must     |
| S-002   | Fault response time shall be faster than one PWM period for overcurrent     | Must     |
| S-003   | The drive shall be operable only after a successful self-check at startup   | Should   |

## 4. Implementation constraints

| ID      | Requirement                                                                 | Priority |
| ------- | --------------------------------------------------------------------------- | -------- |
| C-001   | Control loops executed at a fixed rate; current loop ≥ 10 kHz, synchronized to PWM | Must |
| C-002   | PWM frequency 20 kHz, center-aligned                                        | Must     |
| C-003   | All algorithms first validated in Python simulation                         | Must     |
| C-004   | Target MCU: STM32 (specific part selected in Phase 3)                       | Must     |
| C-005   | Industry-standard free toolchain (no licensed SDK dependencies)         | Must     |

## Performance targets (to be refined in Phase 2)

| Metric                              | Initial target          |
| ----------------------------------- | ----------------------- |
| Speed steady-state accuracy         | ±0.5 % of reference     |
| Speed settling (0 → rated)          | < 100 ms (no load)      |
| Torque response (step to 50 % rated)| < 10 ms                 |
| Current loop bandwidth              | ~1–2 kHz (design goal)  |
| Speed loop bandwidth                | ~50–100 Hz (design goal)|

*Exact values will be confirmed once the reference motor parameters are frozen in `motor-specifications.md`.*
