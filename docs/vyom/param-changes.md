# Parameters we change from PX4 defaults

One row per parameter, with the reason. Values for the real airframe come from bench and flight tests; anything marked `TODO(verify)` is an estimate.

## Simulation model (SIH, `/sim/models/vyom_x249.yaml` in the monorepo)
These only affect the simulator, not a real drone. They describe our sub-250 g X-quad until prompt 08 provides a measured model.

| Param | PX4 default | Ours | Reason |
|---|---|---|---|
| see `/sim/models/vyom_x249.yaml` | | | Values and reasons are kept in that file so the simulator and this table cannot drift; CI prints them. |

## Flight parameters: airframe `4950_vyom_x249` (all boards)
`fw/flight/tools/check_airframe.py` (monorepo) checks that every flight param the sim model sets has the same value here, and that no sim-only or failsafe-disabling param is in this file.

| Param | PX4 default | Ours | Reason |
|---|---|---|---|
| `CA_ROTOR0..3_PX/PY` | airframe-specific | ±0.088 m | Square X, 0.124 m centre to motor. TODO(verify) frame CAD |
| `CA_ROTOR0..3_KM` | 0.05 | ±0.016 | Small-prop torque/thrust ratio, same as the sim model. TODO(verify) thrust stand |
| `MPC_THR_HOVER` | 0.5 | 0.40 | 0.245 kg on 4 × 1.5 N. TODO(verify) after the first hover |
| `THR_MDL_FAC` | 0.0 | 0.3 | Small props: thrust is closer to RPM² than linear. TODO(verify) thrust stand |
| `EKF2_EV_CTRL` | 0 | 5 (horizontal position + velocity) | Fuse the SoC's visual odometry (WP3). Height stays on baro + ToF, yaw on the magnetometer until the SoC's VIO yaw is characterised. NEEDS-SW: revisit when the real VIO exists |
| `EKF2_RNG_A_HMAX` | 5 m | 3.5 m | Use the down ToF for height only below 3.5 m (landing, low hover). TODO(verify) the ToF part's real range over grass and concrete (NEEDS-EE) |
| `MC_ROLLRATE_P` / `MC_PITCHRATE_P` | 0.15 | 0.08 | Stock gains wobble the light model on the ground and block landing detection (WP0 finding). WP4 tunes |
| `MC_ROLLRATE_I` / `MC_PITCHRATE_I` | 0.2 | 0.1 | As above |
| `MC_ROLLRATE_D` / `MC_PITCHRATE_D` | 0.003 | 0.0015 | As above |
| `DSHOT_BIDIR_EN` | 0 | 1 | Our STM32G431 ESCs send RPM back (bidirectional DShot) |
| `MOT_POLE_COUNT` | 14 | 12 | TODO(verify) motor part, NEEDS-EE |
| `IMU_GYRO_DNF_EN` | 0 | 1 | Notch filter that follows the motor RPM: removes prop vibration from the gyro |
| `BAT1_N_CELLS` | board default | 2 | 2S pack. TODO(verify) POWER_BUDGET.csv |
| `MPC_LAND_SPEED` | 0.7 m/s | 0.5 m/s | Light airframe bounces at 0.7 m/s |

## Board defaults: `boards/vyom/core-v1/init/rc.board_defaults`
| Param | PX4 default | Ours | Reason |
|---|---|---|---|
| `SENS_EN_THERMAL` | -1 | 0 | No IMU heater on the board |
| `EKF2_MULTI_IMU` | 0 | 2 | Two IMUs (BMI088 + ICM-42688-P): run one estimator per IMU and pick the healthy one |
| `SENS_IMU_MODE` | 1 | 0 | Required by `EKF2_MULTI_IMU` > 0 |
| `PWM_MAIN_TIM0` | 400 Hz PWM | -3 (DShot600) | ESCs speak DShot only |
| `MAV_0_CONFIG` | TELEM1 | 0 (off) | USART2 to the SoC now runs our `/proto` link instead of MAVLink (WP2). NEEDS-SW: ground-station MAVLink needs a tunnel through the SoC (WP7) |
| `VYM_SOC_CFG` | 0 | 101 (TELEM1) | Starts `vyom_soc_link` on USART2 |
| `SER_TEL1_BAUD` | 57600 | 3000000 | FC↔SoC UART runs at 3 Mbaud (CLAUDE.md) |
| `SYS_DM_BACKEND` | 0 (file) | 1 (RAM) | No SD card; mission storage lives in RAM. NEEDS-SW: missions are lost on reboot |
| `SDLOG_MODE` | 0 | -1 (off) | No SD card; flight logs stream to the SoC instead (WP7). NEEDS-SW |

## Board defaults: `boards/vyom/proto0/init/rc.board_defaults` (ARK FPV)
| Param | PX4 default | Ours | Reason |
|---|---|---|---|
| `MAV_0_MODE` | normal | 2 (onboard) | Companion computer link on TELEM1. TODO(verify) wiring |
| `SER_TEL1_BAUD` | 57600 | 921600 | Companion link speed |

## New Vyom parameters
| Param | Default | Module | Meaning |
|---|---|---|---|
| `VYM_SOC_CFG` | 0 (off) | `vyom_soc_link` | Serial port of the SoC link (core-v1 sets TELEM1) |
| `VYM_SOC_TOUT` | 0.5 s | `vyom_soc_link` | No SoC heartbeat for this long = SoC lost (`vyom_soc_status.soc_alive`). SoC sends at 10 Hz, so 5 missed beats |
| `VYM_WDT_KICK_MS` | 100 ms | `vyom_wdt` | External watchdog kick period. TODO(verify) against the watchdog part's window (NEEDS-EE) |
| `VYM_GG_JUMP` | 5 m | `vyom_gnss_guard` | Reject GNSS when one 10 Hz step is this far from what the EKF (IMU) velocity explains |
| `VYM_GG_VIO_POS` | 3 m | `vyom_gnss_guard` | Reject GNSS when its displacement over the last 10 s differs from the VIO displacement by more than this |
| `VYM_GG_VIO_VEL` | 0.5 m/s | `vyom_gnss_guard` | Reject GNSS when its velocity (1 s low-pass) differs from VIO velocity by more than this |
| `VYM_GG_RECOV` | 10 s | `vyom_gnss_guard` | Clean time before GNSS is accepted again (plus: within 3 m of the EKF with VIO, or landed without VIO) |

## Simulation-only parameters (`vyom_gnss_sim`, never on a real board)
| Param | Default | Meaning |
|---|---|---|
| `VYM_SIM_GF_MODE` | 0 | Injected GNSS fault: 1 jump, 2 drift spoof, 3 jamming, 4 drift spoof flagged by the receiver, 5 drift spoof with uniform C/N0 |
| `VYM_SIM_GF_MAG` | 50 | Jump size (m) or drift speed (m/s) |
| `VYM_SIM_GF_DIR` | 90° | Direction of the jump or drift, from north |
