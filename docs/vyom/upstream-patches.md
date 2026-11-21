# Edits to upstream PX4 files

Every change outside `src/modules/vyom_*`, `src/drivers/vyom_*`, `boards/vyom/*`, `docs/vyom/*`, `Tools/vyom/*` and `.github/workflows/vyom-*.yml` is listed here. Commits carry the prefix `upstream-patch:`.

| # | File(s) | Change | Why | Upstreamable? | Since |
|---|---|---|---|---|---|
| 1 | `ROMFS/px4fmu_common/init.d/airframes/4950_vyom_x249` (new) + 1 line in that folder's `CMakeLists.txt` | Vyom X249 airframe (SYS_AUTOSTART 4950) | PX4 only loads airframes from this folder; a new file conflicts with nothing on rebase. The CMakeLists line sits after `4901_crazyflie21` | No (our airframe) | `v1.17.0`, WP1 |
| 2 | `msg/VyomSocStatus.msg` (new) + 1 line in `msg/CMakeLists.txt` | uORB topic `vyom_soc_status` (SoC link health) | PX4 only builds uORB messages listed in that file; a new file conflicts with nothing on rebase | No (our topic) | `v1.17.0`, WP2 |
| 3 | `msg/VyomGnssStatus.msg` (new) + 1 line in `msg/CMakeLists.txt`; `src/modules/ekf2/EKF2.hpp`, `EKF2.cpp` (`UpdateGpsSample`, marked `vyom: upstream-patch #3`) | EKF2 treats a GNSS sample as spoofed when `vyom_gnss_status.reject` is set (OR with the receiver's own spoofing flag). A verdict older than 1 s is ignored, so a dead guard cannot block GNSS | Lets `vyom_gnss_guard` stop GNSS fusion through EKF2's existing spoofing path (samples skipped at once, fusion stopped after EKF2's own timeout) without touching the filter | Maybe: a generic "external GNSS integrity" input could be proposed upstream | `v1.17.0`, WP3 |
