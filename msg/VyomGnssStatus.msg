# Vyom: GNSS integrity verdict from vyom_gnss_guard. EKF2 stops GNSS fusion while reject is true
# (docs/vyom/upstream-patches.md #3).

uint64 timestamp                    # [us] Time since system start

bool reject                         # GNSS must not be fused (latched until recovery conditions hold)
uint16 reasons                      # Bitmask of checks failing now (REASON_*)
uint16 reasons_latched              # Bitmask of checks that caused the current rejection
uint16 REASON_RX_SPOOF   = 1        # Receiver reports spoofing detected
uint16 REASON_RX_JAM     = 2        # Receiver reports jamming detected
uint16 REASON_JUMP       = 4        # Position step not explained by GNSS or EKF velocity
uint16 REASON_VIO_POS    = 8        # GNSS displacement disagrees with visual odometry displacement
uint16 REASON_VIO_VEL    = 16       # GNSS velocity disagrees with visual odometry velocity
uint16 REASON_CN0        = 32       # Satellite signal strengths look spoofed (too uniform and strong)

float32 jump_m                      # [m] Last unexplained position step
float32 vio_pos_diff_m              # [m] |GNSS - VIO| displacement over the comparison window, NaN if no VIO
float32 vio_vel_diff_m_s            # [m/s] Filtered |GNSS - VIO| horizontal velocity, NaN if no VIO
float32 cn0_mean_dbhz               # [dB-Hz] Mean C/N0 of used satellites, NaN if unknown
float32 cn0_std_dbhz                # [dB-Hz] Std dev of C/N0 of used satellites, NaN if unknown
uint32 reject_count                 # Number of times GNSS has been rejected since boot
