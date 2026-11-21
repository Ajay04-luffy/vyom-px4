# Vyom X249 tuning guide (first real flights)

Who: SW flies, EE runs the bench and power side. Nothing here happens before the owning
co-founder has approved the hardware and firmware PRs (gates G1–G7). Read the safety rules first.

The airframe `4950_vyom_x249` ships a **baseline** derived from the simulator model
(`/sim/models/vyom_x249.yaml` in the monorepo). The model's mass, inertia, thrust and drag are
estimates, so the baseline is a starting point that should fly, not a tune.

## 0. Safety rules
- Props **off** for every step until "first hover". Remove props before changing motor order,
  direction or ESC settings.
- First hovers tethered or in a net, 1–2 m high, at least 5 m from people, battery in a fire-safe
  bag nearby. Two people: pilot and someone on the kill switch.
- Arm/disarm and the kill switch on the remote must be tested with props off before every session.
- Change one thing at a time, save the log of every flight (logs stream to the SoC; WP9).

## 1. Measure the real airframe (EE, bench)
| What | How | Updates |
|---|---|---|
| Takeoff mass | scale, flight-ready with battery | `SIH_MASS`, `WEIGHT_BUDGET.csv` |
| Thrust curve per motor + prop | thrust stand: thrust (N) and RPM at 10 throttle steps | `SIH_T_MAX`, `THR_MDL_FAC`, `MPC_THR_HOVER` |
| Motor time constant | step the throttle on the stand, log RPM (bidirectional DShot); time to 63 % | `SIH_T_TAU` |
| Inertia | bifilar pendulum, or from CAD with real part masses | `SIH_IXX/IYY/IZZ` |

**Thrust curve to `THR_MDL_FAC`:** normalise thrust to 0–1 (divide by the maximum) and fit
`thrust = f·u² + (1 − f)·u` over throttle `u` in 0–1. `f` is `THR_MDL_FAC`. Small props are
usually between 0.3 and 0.5; the baseline guesses 0.3.

**Hover thrust:** `MPC_THR_HOVER` ≈ throttle where thrust = mass × 9.81 / 4 per motor. In flight,
PX4's estimator refines it (`hover_thrust_estimate` in the log); copy the logged value back.

Then update the sim model with the measured values, re-run the step scenarios
(`sim/scenarios/step_*.yaml`), and re-derive the gains below before flying.

## 2. Motors, RPM telemetry and the dynamic notch (props off, then on)
1. `DSHOT_BIDIR_EN 1` and `MOT_POLE_COUNT` (baseline 12, **check the motor datasheet**, NEEDS-EE).
   With props off, spin each motor with `actuator_test` and check `esc_status` shows a plausible
   RPM for each motor.
2. `IMU_GYRO_DNF_EN 1` (bit 0 = notch per motor from ESC RPM). Leave `IMU_GYRO_DNF_HMC 3`
   (3 harmonics), `IMU_GYRO_DNF_BW 15` Hz and `IMU_GYRO_DNF_MIN 25` Hz at their defaults.
3. After the first hover, open the log in PX4 Flight Review and look at the gyro FFT. The motor
   peaks should be gone; if a peak remains, it is a frame resonance (fix the mount, or add a
   static notch `IMU_GYRO_NF0_FRQ`).
4. Only once the FFT is clean, raise the gyro low-pass `IMU_GYRO_CUTOFF` (default 40 Hz) in
   10 Hz steps, watching motor temperature and the FFT. Small quads often end at 60–90 Hz.

The simulator has no vibration, so nothing in this section is tested in SIH.

## 3. First hover (props on, tethered)
- Mode ALTCTL first, then POSCTL. Hover 30 s at 1.5 m.
- Check in the log: `hover_thrust_estimate` (copy to `MPC_THR_HOVER`), motor outputs (none
  near 0 or 1 at hover), and no oscillation in `vehicle_angular_velocity`.
- If it oscillates fast (a buzz): lower `MC_ROLLRATE_P/PITCHRATE_P` by 20 % and `..._D` by 20 %.
  If it wobbles slowly: lower `MC_ROLL_P/PITCH_P` towards 6.5.

## 4. Rate loop (the inner loop)
Preferred: PX4's autotune (`mc_autotune_attitude_control`, in the core-v1 build).
- Hover in POSCTL at 5 m with space around, set `MC_AT_EN 1`, then start it (`MC_AT_START 1`
  or the remote's autotune switch once that exists). It injects small excitations for ~60 s per
  axis and proposes rate and attitude gains. Accept only after landing, then fly and compare.
- Manual alternative: step roll/pitch with the sticks in STABILIZED and tune P until a small
  overshoot, D to damp it, I last (should hold attitude in wind without drifting).

## 5. Attitude, velocity and position loops
Same acceptance limits as the simulator scenarios, checked from real logs:

| Loop | Test | Target (sim baseline result) | Limit |
|---|---|---|---|
| Roll/pitch attitude | 15° step | rise 0.20 s, overshoot 1–2 % | rise < 0.3 s, overshoot < 15 %, settle < 0.5 s |
| Yaw | 45° step | rise 0.78 s, overshoot 2 % | rise < 1.0 s, overshoot < 10 % |
| Horizontal velocity | 3 m/s step | rise 0.8–1.0 s, overshoot ≤ 2 % | rise < 1.2 s, overshoot < 15 %, settle < 2 s |
| Margin | slower motors, +30 % inertia | rise 0.16–0.18 s, overshoot ≤ 2 % | stays stable |

- `MC_ROLL_P/PITCH_P 9` (stock 6.5): faster attitude. Lower it if the real drone overshoots.
- `MPC_XY_VEL_P_ACC 2.5`, `MPC_XY_VEL_I_ACC 2.0` (stock 1.8 / 0.4): the sim model's drag
  (`SIH_KDV 0.25`) is high, and with stock gains a 3 m/s step stalled at 80 %. If the real drag is
  lower, these gains may overshoot: lower I first.
- Vertical velocity: stock gains kept. In SIH, climb-rate steps show a run-to-run bias of
  about 0.25 m/s that is not understood yet (NEEDS-SW), so check climb and descent on the first
  flights against the barometer and GNSS height.

## 6. What to record for the next model update
Mass, thrust-stand table, motor time constant, final gains, `hover_thrust_estimate`, and the
gyro FFT screenshot. They go into `/sim/models/vyom_x249.yaml` (prompt 08 builds the measured
model), so the simulator and the real drone stay close.
