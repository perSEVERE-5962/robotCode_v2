# Dashboard Reference

Two tools, two purposes:
- **Elastic Dashboard** -- live, behind the glass, for the driver (3 layouts, 7 tabs)
- **AdvantageScope** -- post-match, in the pit, for engineers (9 layouts)

---

## Folder Structure

```
ref/dashboards/
├── README.md                              <- this file
├── elastic/                               <- Elastic: designs + JSON copies
│   ├── LAYOUTS.md                         <- Layout reference & tips
│   ├── driver_competition_design.md       <- Competition dashboard spec
│   ├── rebuilt_driver_competition.json     <- 2 tabs: Match + Coach
│   ├── pit_diagnostic_design.md           <- Pit diagnostic spec
│   ├── rebuilt_pit_diagnostic.json         <- 3 tabs: Quick Check + Diagnostics + System Detail
│   └── rebuilt_tuning_session.json         <- 2 tabs: Shooter + Subsystems
└── advantagescope/                         <- AdvantageScope: importable .json
    ├── LAYOUTS.md                         <- Layout reference & troubleshooting guide
    ├── match_review.json                  <- Post-match debrief (scoring, cycles, field position)
    ├── cycle_and_strategy.json            <- Hub strategy, cycle timing, intake pipeline
    ├── pit_triage.json                    <- Quick pit health check between matches
    ├── mechanism_debug.json               <- Mechanism PID debugging (shooter/indexer/hanger)
    ├── power_and_health.json              <- Brownout/CAN/loop analysis
    ├── drive_and_auto.json                <- Swerve drive + auto path analysis
    ├── shooter_tuning.json                <- Shooter PID tuning (11 tabs)
    ├── system_overview.json               <- General health check + post-match summary
    └── vision_debug.json                  <- Vision target tracking
```

**Deployed Elastic JSONs** live in `robotcode2026-lab/src/main/deploy/elastic/` (authoritative for robot). Copies here for reference.

**Archived V1-V5 role-specific dashboards** (over-engineered, 56-97 widgets each) in `ref/archive/dashboards_role_specific_20260205/`.

---

## Elastic Dashboard (3 layouts, 7 tabs)

Real-time dashboard on the driver station monitor. Keep it simple -- drivers view from 3-6 feet under match stress.

See [elastic/LAYOUTS.md](elastic/LAYOUTS.md) for details and tips.

| Layout | JSON File | Tabs | Audience |
|--------|-----------|------|----------|
| Competition (v3) | `rebuilt_driver_competition.json` | **Match** (10 widgets) + **Coach** (12 widgets) | Driver + coach |
| Pit Diagnostic (v3) | `rebuilt_pit_diagnostic.json` | **Quick Check** (20) + **Diagnostics** (15) + **System Detail** (19) | Pit crew |
| Tuning (v3) | `rebuilt_tuning_session.json` | **Shooter** (30, RPM Graph + ShotPredictor) + **Subsystems** (19, Current Graph) | Programmer |

**Key widget upgrades in v3:** Match Time (color-coded timer), Voltage View (battery), Graph (time-series for PID tuning), Large Text Display (shot count for coach), Boolean Box for CAN Bus Off.

---

## AdvantageScope (9 layouts)

Post-match log analysis. Import via **File > Import Layout > select .json**.

See [advantagescope/LAYOUTS.md](advantagescope/LAYOUTS.md) for details and troubleshooting guide.

### Workflow Layouts (use between matches)

| Layout | File | Tabs | When to Use |
|--------|------|------|-------------|
| Match Review | [match_review.json](advantagescope/match_review.json) | 7 | After every match -- scoring timeline (heading error + prediction active), 3D field with shot arc, cycles, driver inputs |
| Cycle & Strategy | [cycle_and_strategy.json](advantagescope/cycle_and_strategy.json) | 5 | Hub shift analysis, cycle timing, intake-to-score pipeline, scoring positions |
| Pit Triage | [pit_triage.json](advantagescope/pit_triage.json) | 5 | Quick pit health check -- battery, faults, performance, field replay |
| Mechanism Debug | [mechanism_debug.json](advantagescope/mechanism_debug.json) | 8 | Shooter/indexer/intake/hanger PID response, jams, temps |
| Power & Health | [power_and_health.json](advantagescope/power_and_health.json) | 6 | Brownout, CAN errors, loop overruns, thermals, crash barriers |

### Utility Layouts (focused debugging)

| Layout | File | Tabs | Focus |
|--------|------|------|-------|
| Drive & Auto | [drive_and_auto.json](advantagescope/drive_and_auto.json) | 6 | Swerve pose, speeds, path error, gyro, modules |
| Shooter Tuning | [shooter_tuning.json](advantagescope/shooter_tuning.json) | 12 | Velocity, PID, shot detection, fire rate, recovery time, shot prediction, drift compensation |
| System Overview | [system_overview.json](advantagescope/system_overview.json) | 10 | Battery, CAN, loop timing, temps, alerts, post-match summary |
| Vision Debug | [vision_debug.json](advantagescope/vision_debug.json) | 6 | Target lock, latency, consecutive frames, target offset |

---

## Quick Reference

| Problem | Tool | Layout |
|---------|------|--------|
| Match in progress | Elastic | Competition > Match |
| Coach strategy | Elastic | Competition > Coach |
| Pre-match check | Elastic | Pit Diagnostic > Quick Check |
| Run diagnostics | Elastic | Pit Diagnostic > Diagnostics |
| Deep system check | Elastic | Pit Diagnostic > System Detail |
| Tune shooter PID | Elastic | Tuning > Shooter (Graph) |
| Tune other subsystems | Elastic | Tuning > Subsystems |
| "Did we score enough?" | AdvantageScope | match_review |
| "Are our cycles fast?" | AdvantageScope | cycle_and_strategy (Tab 2) |
| "Hub strategy working?" | AdvantageScope | cycle_and_strategy (Tab 1) |
| "Quick pit health check" | AdvantageScope | pit_triage |
| "Auto went sideways" | AdvantageScope | drive_and_auto (Tab 3) |
| "Shooter won't spin up" | AdvantageScope | mechanism_debug (Tab 1) |
| "Indexer keeps jamming" | AdvantageScope | mechanism_debug (Tab 3) |
| "We browned out" | AdvantageScope | power_and_health (Tab 1) |
| "CAN errors in DS log" | AdvantageScope | power_and_health (Tab 2) |
| "Swerve is drifting" | AdvantageScope | drive_and_auto (Tab 5) |
| "Tune shooter PID" | AdvantageScope | shooter_tuning |
| "Shot prediction working?" | AdvantageScope | shooter_tuning (Tab 10-11) |
| "Heading off when shooting?" | AdvantageScope | match_review (Tab 1) |
| "Vision dropping targets" | AdvantageScope | vision_debug |

---

*12 dashboards total (3 Elastic / 7 tabs + 9 AdvantageScope) covering ~425 telemetry signals*

## Signal Validation Notes

All dashboard signal paths were audited against actual `SafeLog.put()` and `Logger.recordOutput()` calls in the codebase (Feb 5, 2026). Key findings:

- **~425 actual signals** in code (more than the ~240 previously documented)
- **Phantom signals removed:** `SystemHealth/HeapUsagePercent`, `Drive/ChassisSpeeds/Requested/*`
- **Naming fixes applied:** `Drive/Gyro/Yaw` -> `Drive/Gyro/YawRad`, chassis speed path corrections
- **Known stubs:** `Scoring/Conditions/HasBall` (always true -- no ball sensor hardware)
- **HubActive unstubbed:** Now computed from match timer (Session 18), added to driver competition dashboard
- **v5.1 signals added:** Shooter/Indexer PID audit trails, mechanism health indicators, driver coaching signals
- **Dashboard updates:** HubActive on competition, gyro/camera/latency on pit, yaw/pitch on tuning, requested speeds + connectivity on AdvantageScope layouts
