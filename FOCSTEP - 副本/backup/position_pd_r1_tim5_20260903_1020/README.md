# Pulse-position PD baseline

- Snapshot time: 2026-09-03 10:20 (Asia/Shanghai)
- Firmware identity: `PULSE mode=position_pd_r1`
- STEP input: PC7 / TIM3_CH2 hardware capture
- Outer loop: TIM5 at 4 kHz
- Control law in pulse mode: `SpeedRef = 2.4 * (PosRef - PosFb) - 0.05 * SpeedFb`
- Pulse frequency is diagnostic only; position increments exactly 5 encoder counts per STEP.
- The matching compiled image is `MDK-ARM/FOCSTEP/FOCSTEP.hex`.
