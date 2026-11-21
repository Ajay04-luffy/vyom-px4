# Parameters we change from PX4 defaults

One row per parameter, with the reason. Values for the real airframe come from bench and flight tests; anything marked `TODO(verify)` is an estimate.

## Simulation model (SIH, `/sim/models/vyom_x249.yaml` in the monorepo)
These only affect the simulator, not a real drone. They describe our sub-250 g X-quad until prompt 08 provides a measured model.

| Param | PX4 default | Ours | Reason |
|---|---|---|---|
| see `/sim/models/vyom_x249.yaml` | | | Values and reasons are kept in that file so the simulator and this table cannot drift; CI prints them. |

## Flight parameters
None yet. Board ports (WP1) and tuning (WP4) add rows here.
