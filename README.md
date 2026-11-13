# CHAMS Industrial Motor Drive

Development of an industrial-oriented three-phase PMSM motor drive using Field-Oriented Control (FOC), featuring motor simulation, torque and speed regulation, and future STM32 embedded implementation.

**Approach:** simulation first · zero budget · proprietary development
**Status:** Phase 1 — Industrial Motor Definition and Mathematical Modelling

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
│   └── references.md
│
├── diagrams/
├── simulation/
└── firmware/
```

## Documentation

- [Project overview](docs/project-overview.md)
- [System requirements](docs/system-requirements.md)
- [Motor specifications](docs/motor-specifications.md)
- [Motor mathematical model](docs/motor-mathematical-model.md)
- [References](docs/references.md)

## Phase roadmap

- **Phase 1** — Industrial motor definition and mathematical modelling *(current)*
- **Phase 2** — Simulation of the FOC drive loop
- **Phase 3** — STM32 firmware implementation

## License

[MIT](LICENSE)
