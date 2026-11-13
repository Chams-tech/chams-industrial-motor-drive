# CHAMS Industrial Motor Drive

Development of an industrial-oriented three-phase PMSM motor drive using Field-Oriented Control (FOC), featuring motor simulation, torque and speed regulation, and future STM32 embedded implementation.

**Approach:** simulation-first · proprietary development
**Status:** Phase 2 — PMSM FOC simulation (complete, validated)

## Design direction

| Decision            | Choice                                    |
| ------------------- | ----------------------------------------- |
| Intended domain     | Industrial robotics and automation        |
| Target power        | Below 500 W                               |
| Simulation DC bus   | 48 V                                      |
| Motor technology    | Three-phase PMSM (surface-mounted magnets)|
| Control technique   | FOC (speed + torque control)              |
| Position feedback   | Encoder                                   |
| Future controller   | STM32                                     |

## Repository structure

```
chams-industrial-motor-drive/
│
├── README.md
│
├── docs/
│   ├── project-overview.md
│   ├── system-requirements.md
│   ├── motor-specifications.md
│   ├── motor-mathematical-model.md
│   ├── PHASE_2.md
│   └── references.md
│
├── simulation/
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
├── results/          # generated plots (7 scenarios × 6 plots)
├── diagrams/
├── firmware/
├── run_phase2.py     # scenario runner
└── requirements.txt
```

## Documentation

- [Project overview](docs/project-overview.md)
- [System requirements](docs/system-requirements.md)
- [Motor specifications](docs/motor-specifications.md)
- [Motor mathematical model](docs/motor-mathematical-model.md)
- [Phase 2 — FOC simulation](docs/PHASE_2.md)
- [References](docs/references.md)

## Running the simulation

```bash
pip install -r requirements.txt
python -m pytest tests/        # unit tests (22)
python run_phase2.py           # all 7 scenarios + plots in results/
python run_phase2.py startup   # single scenario
```

## Phase roadmap

- **Phase 1** — Industrial motor definition and mathematical modelling ✅
- **Phase 2** — Simulation of the FOC drive loop ✅
- **Phase 3** — STM32 firmware implementation *(next)*

## License

[MIT](LICENSE)
