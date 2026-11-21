# Vyom PX4 fork policy

Repo: `Ajay04-luffy/vyom-px4` (private). Monorepo path: `/fw/px4` (git submodule).
Owner: **SW**. Decision record: `/docs/adr/0005-px4-fork-base-and-hosting.md` in the monorepo.

## Branches and tags
| Ref | Meaning |
|---|---|
| `upstream/*` | PX4 as published (`https://github.com/PX4/PX4-Autopilot`). Never commit here. |
| `vyom/main` | Our product branch. Starts at an upstream **stable** tag (today `v1.17.0`). |
| `vyom/rebase-<tag>` | Temporary branch while moving to a new upstream tag. |
| `vyom-v<upstream>-<n>` | Our release tags, e.g. `vyom-v1.17.0-1`. |

## Where our code goes (keeps rebases cheap)
1. **Own folders only:** `src/modules/vyom_*`, `src/drivers/vyom_*`, `boards/vyom/*`, `docs/vyom/*`, `Tools/vyom/*`, `.github/workflows/vyom-*.yml`.
2. **Configuration before code:** behaviour change by parameters, board files and airframe files first; a new module second; an edit to an upstream file last.
3. **Every edit to an upstream file** (any path outside the folders above) is listed in [`upstream-patches.md`](upstream-patches.md) with the reason, the file, and whether it can go upstream. Keep each such edit in its own commit with the prefix `upstream-patch:` so it is easy to find during a rebase.
4. **Parameters** we change from PX4 defaults are listed in [`param-changes.md`](param-changes.md) with a reason.
5. **No GPL code** (e.g. ArduPilot) is copied or translated into the fork. PX4 is BSD-3; our changes may stay private, but we keep PX4's copyright headers and licence file.

## Rebasing (quarterly)
1. Pick the newest upstream **stable** tag (no `-rc`, `-beta`, `-alpha`).
2. `git fetch upstream --tags && git checkout -b vyom/rebase-<tag> vyom/main && git rebase --onto <tag> <old-base-tag>`.
3. Resolve conflicts; re-check every entry in `upstream-patches.md` (drop ones upstream now covers).
4. CI must be green: all Vyom targets build, PX4 unit tests, our unit tests, full `/sim` scenario suite.
5. SW reviews; merge by resetting `vyom/main` to the rebase branch (force-push is allowed **only** for this step, by SW), then bump the submodule pointer in the monorepo with a PR.
6. A flight build from a new base is bench-tested and test-flown again under gate rules before it reaches any user.

## Security fixes between rebases
- Watch PX4 releases and GitHub security advisories for `PX4/PX4-Autopilot` and NuttX (SW subscribes; weekly check is fine).
- A fix that touches flight safety, MAVLink parsing, the bootloader or any external interface is cherry-picked onto `vyom/main` within **2 weeks** and noted in `upstream-patches.md` as "backport".

## Contributing back
Generic fixes and drivers (not our protocols, not product behaviour) may be offered upstream as normal PX4 PRs once SW agrees. Nothing that reveals product plans goes upstream.

## Enabling Vyom modules
Vyom modules use `default y if PLATFORM_POSIX` in their `Kconfig`, so SITL and the unit-test build include them **without editing upstream board files**. NuttX boards enable them explicitly in `boards/vyom/*/default.px4board`.
