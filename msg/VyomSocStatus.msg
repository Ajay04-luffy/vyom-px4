# Vyom: health of the FC<->Vision SoC link (published by vyom_soc_link).
# WP6 failsafes act on soc_alive. See monorepo /proto/vyom_link_v0.yaml for the wire format.

uint64 timestamp                # [us] Time since system start

bool soc_alive                  # SoC heartbeat seen within VYM_SOC_TOUT
uint64 last_heartbeat           # [us] FC time of the last valid SoC heartbeat, 0 if never
uint8 soc_health                # SoC self-reported health: 0 boot, 1 ok, 2 degraded, 3 fault
uint8 soc_schema_minor          # /proto minor version the SoC was built with
uint32 soc_uptime_ms            # [ms] SoC uptime from its last heartbeat

bool timesync_converged         # SoC clock offset estimate is usable
int64 timesync_offset_us        # [us] FC time minus SoC time

uint32 frames_ok                # Valid frames received
uint32 crc_errors               # Frames dropped: CRC mismatch
uint32 framing_errors           # Frames dropped: bad COBS, bad length, overflow
uint32 version_errors           # Frames dropped: wrong /proto major version
uint32 rejected_msgs            # Valid frames dropped: unknown id, wrong direction or short payload
uint32 odometry_dropped         # Vision odometry dropped: no timesync yet or quality 0
uint32 setpoints_ignored        # Trajectory setpoints received but not used (WP5 adds the mode that uses them)
uint32 tx_overruns              # Frames not sent because the UART TX buffer was full
