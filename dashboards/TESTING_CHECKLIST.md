# Testing Checklist

This branch adds telemetry logging and 12 dashboard layouts. **The robot drives exactly the same as before.** Telemetry is read-only, it just records what's happening. It can't change how anything moves.

If something drives differently after deploying this, it's not the logging. Check if something else changed.

```mermaid
flowchart LR
    A[Robot] -->|logs signals| B[.wpilog file on USB]
    A -->|live via NetworkTables| C[Elastic on DS laptop]
    B -->|open after match| D[AdvantageScope on laptop]
```

---

## Bare minimum test (2 min)

If you only do one thing, do this:

1. `./gradlew build` → should say BUILD SUCCESSFUL
2. `./gradlew deploy` → robot boots, no errors in Driver Station
3. Drive around for 30 seconds → **robot should feel exactly the same as before**
4. Check `SystemHealth/BatteryVoltage` in AdvantageScope → should read 12-13V

If all four pass, logging is working and nothing is broken. Everything below is extra verification.

---

## Install the dashboard tools

You need both of these on the driver station laptop. If you already have them, make sure you're on the 2026 versions.

**AdvantageScope** (post-match log viewer):
- Download: https://github.com/Mechanical-Advantage/AdvantageScope/releases
- Grab the latest v26.x `.exe` installer for Windows
- This is what we use in the pit to review match logs

**Elastic Dashboard** (live match display):
- Download: https://github.com/Gold872/elastic-dashboard/releases
- Grab the latest 2026 release for Windows
- This is what goes on the driver station monitor behind the glass

---


## Loading the dashboards

### AdvantageScope

1. Open AdvantageScope
2. To view live: **File > Connect to Robot** (or drag a `.wpilog` file in for post-match)
3. **File > Import Layout**
4. Pick a `.json` from `dashboards/advantagescope/`
5. Good ones to start with:
   - **match_review.json** for scoring timeline, field replay, cycle times after a match
   - **pit_triage.json** for quick health checks between matches (battery, faults, temps)
6. The layout loads all the tabs at once. Just click through them.

### Elastic

1. Open Elastic Dashboard
2. It should auto-connect to the robot if you're on the same network (check the connection indicator in the top bar)
3. **File > Open Layout**
4. Pick a `.json` from `dashboards/elastic/`
5. Start with **rebuilt_driver_competition.json**, which has a Match tab for the driver and a Coach tab for strategy

See `dashboards/README.md` for what all 12 layouts do.

---

## Quick verify (5 min, robot on)

1. Open AdvantageScope and connect to the robot
2. Expand `/RealOutputs/` in the sidebar
3. You should see folders like `Shooter/`, `Drive/`, `Vision/`, `Scoring/`, `SystemHealth/`
4. Check these signals:
   - `Shooter/VelocityRPM` should be non-zero when the shooter is spinning
   - `Drive/Pose` should update when you push the robot around
   - `SystemHealth/BatteryVoltage` should read around 12-13V
   - `Scoring/ReadyToShoot` goes true when all 5 conditions are met

If you see all of those, telemetry is working.

---

## Test matrix (10 min)

Run through these with the robot enabled. Check the signals in AdvantageScope as you go.

**Important:** Some mechanisms might not be on the robot yet. That's fine. Skip the ones marked "skip if not built" and come back to them later. Logging still works for everything else.

| # | Test | What to do | What you should see | Hardware needed? |
|---|------|------------|---------------------|-----------------|
| 1 | Build | `./gradlew build` | BUILD SUCCESSFUL | No |
| 2 | Deploy | `./gradlew deploy` | Robot boots, no errors in Driver Station | Robot on |
| 3 | Drive | Drive around for 30 sec | `Drive/Pose` updates, robot feels normal | Drivetrain |
| 4 | Shooter | Spin up the shooter | `Shooter/VelocityRPM` goes up, `Shooter/AtSpeed` goes true | Shooter motor |
| 5 | Intake | Run the intake | `Intake/Running` goes true | **Skip if intake not built** |
| 6 | Vision | Point at an AprilTag | `Vision/HasTarget` goes true | **Skip if cameras not mounted** |
| 7 | Scoring | Do a full shoot cycle | `Scoring/ReadyToShoot` goes true when everything lines up | Shooter + vision + indexer all working |
| 8 | Battery | Just check | `SystemHealth/BatteryVoltage` reads a reasonable number | Robot on |
| 9 | Hub timing | Sit in teleop for a minute | `Scoring/HubShiftNumber` should increment every ~25 seconds | Robot enabled in teleop |
| 10 | Health | Check after all tests | `Health/Telemetry/Failures` should still be 0 | Robot on |
| 11 | Dashboard | Import match_review.json into AdvantageScope | All tabs show up and signals populate | Log file from any test above |
| 12 | Live tuning | Load `rebuilt_tuning_session.json` in Elastic, see below | RPM graph updates live, PID changes take effect immediately | Shooter motor |

**Signals that will be zero if hardware isn't installed yet (this is expected, not a bug):**
- `Intake/*` and `IntakeActuator/*` → zero until intake is assembled
- `Hanger/*` → zero until hanger is mounted
- `Vision/*` → zero until cameras are on the robot
- Motor temperatures → zero in the first few seconds, they take time to read

---

## Try live PID tuning

The Tuning layout in Elastic lets you change PID gains on the fly without re-deploying.

1. Load `rebuilt_tuning_session.json` in Elastic
2. Go to the **Shooter** tab
3. The RPM Graph in the middle shows shooter velocity in real time (5-second rolling window)
4. The kP/kI/kD/FF text fields next to it are **editable**. Type a new value and hit Enter.
5. The gain updates instantly on the robot through NetworkTables. You'll see the response change on the graph right away.
6. Same thing works on the **Subsystems** tab for indexer gains, hanger gains, intake speed, and driver deadband

This is way faster than the old edit-build-deploy loop for tuning. Just spin up the shooter, tweak a gain, watch the graph, repeat.

---

## Upload log files for Tamara

After testing, please grab the log file from the roboRIO and upload it so I can review from home.

**Getting the log file:**
```bash
scp admin@10.59.62.2:/home/lvuser/logs/*.wpilog .
```
Or if scp isn't working, connect a USB drive to the roboRIO and copy the `.wpilog` files off it.

**Where to put it:**
Upload to the team Google Drive under `Software/match-logs/`. Name it something like `2026-02-XX_testing_telemetry.wpilog` so I can find it.

I'll review them in AdvantageScope at home and ping you if anything looks off.

---

## If something seems off

| Problem | What to check |
|---------|---------------|
| Robot drives differently | Telemetry is read-only so this shouldn't happen. Check if something else changed. |
| Everything is zeros in AdvantageScope | Check CAN wiring. Try re-deploying. Make sure AdvantageScope is connected. |
| Signals are zero for one mechanism | Is that mechanism built and wired? If not, zeros are expected. |
| `Health/Telemetry/Failures` is above 0 | Look at `Health/Telemetry/LastFailed` to see which telemetry class is having problems. |
| Loop time over 20ms | Check `SystemHealth/LoopTimeMs`. If it stays high, CAN bus might be overloaded. |
| Dashboard file won't open | Make sure you have AdvantageScope v26+ or Elastic 2026. Older versions won't work. |
| Controller isn't vibrating | Make sure `DriverFeedback.initialize()` is being called. Check that hapticScale isn't set to 0. |
