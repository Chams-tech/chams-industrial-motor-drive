# Project Overview

## Objective

Develop an industrial-oriented three-phase PMSM motor controller using Field-Oriented Control (FOC), with a future STM32 implementation.

## Development principles

| Principle            | Meaning                                                     |
| -------------------- | ----------------------------------------------------------- |
| Simulation first     | Every control algorithm is validated in simulation before any firmware is written |
| Toolchain discipline | Industry-standard free toolchain (Python scientific stack, STM32 HAL/CubeIDE) |
| Proprietary development | All control code written in-house; no vendored FOC libraries |

## Target domain

**Industrial robotics and automation** — the drive must deliver:

- fast torque response for motion segments
- tight speed regulation under load disturbances
- precise low-speed operation with encoder feedback
- safe fault handling suitable for machinery

## Target power class

**Below 500 W** — small servo/automation class motors. Chosen because it is:

- easy to simulate with reasonable parameter accuracy
- compatible with a low-cost 48 V bench supply and later hardware
- representative of a large class of industrial automation drives

## Electrical architecture

- 48 V DC bus (initial simulation value)
- two-level three-phase voltage source inverter
- 20 kHz PWM, sinusoidal modulation with SVPWM
- phase current sensing (three-shunt or single-shunt, to be decided in Phase 2)
- incremental encoder for rotor position/speed

## Phase roadmap

| Phase | Content                                              | Status      |
| ----- | ---------------------------------------------------- | ----------- |
| 1     | Motor definition, operating requirements, mathematical model | **current** |
| 2     | FOC simulation: current loops, speed loop, SVPWM     | planned     |
| 3     | STM32 firmware, real-time implementation, protection | planned     |

## Current deliverables

- `docs/system-requirements.md` — functional and safety requirements
- `docs/motor-specifications.md` — reference motor electrical/mechanical parameters
- `docs/motor-mathematical-model.md` — dq-model equations (next)
