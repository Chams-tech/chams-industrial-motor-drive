# References

## Primary reading

1. **IPMSM — physics of anisotropy and optimal control** (Russian),
   Habr — `https://habr.com/ru/articles/1032076/`
   Background on synchronous machines, dq-model and optimal current trajectories;
   used as theoretical reference for the FOC formulation (SPM is the L_d ≈ L_q special case).

## Classic machine/drive texts

2. P. C. Krause, O. Wasynczuk, S. D. Sudhoff, *Analysis of Electric Machinery and Drive Systems*, 3rd ed., Wiley, 2013.
3. N. Mohan, *Electric Drives: An Integrative Approach*, Wiley — FOC derivation and cascaded-loop design.
4. P. Vas, *Sensorless Vector and Direct Torque Control*, Oxford University Press, 1998.

## Application notes (FOC practice)

5. Texas Instruments, *Field Oriented Control of Permanent Magnet Synchronous Motors* (SLUA088 / motor control library docs).
6. STMicroelectronics, *AN543 — Field-oriented control of PMSM with STM32*.
7. STMicroelectronics, *UM1596 — Motor control firmware (MCSDK) principles* (reference for structure, not code reuse).

## Standards and safety context

8. IEC 61800-5-1 — Safety requirements for adjustable speed electrical power drive systems (basis for the fault-handling requirements in `system-requirements.md`).
9. IEC 60034-5 — Degrees of protection (IP code) for rotating electrical machines (context only).

## Tools

10. NumPy / SciPy / Matplotlib — Python simulation stack (Phase 2).
11. STM32CubeIDE + HAL — target toolchain (Phase 3, zero-cost tier).

> Note: all control software in this repository is written in-house; application notes are used for conceptual reference only.
