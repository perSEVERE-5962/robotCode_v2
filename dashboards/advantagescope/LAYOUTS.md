# AdvantageScope Layout Reference

9 pre-built layouts for post-match log analysis. Import via **File > Import Layout**.

---

## Workflow Layouts (use between matches)

### match_review.json: "What happened that match?"
**When:** After every match, first thing you open. 2-minute review.
**Tabs:**
1. **Scoring Timeline**: HeadingErrorRad (left axis) + TimeToNextShift/HubShiftNumber (right axis). Boolean overlay: ReadyToShoot, ShotDetected, AtSpeed, VisionLocked, JamDetected, HubActive, ShotPredictor/Active. Scrub to see exactly when scoring conditions were met or broken, including hub active windows, shift transitions, and whether shot prediction was engaged and heading was accurate.
2. **Field Position**: 3D field replay. Watch robot path, spot where it got stuck or drifted.
3. **Cycle Performance**: Cycle times and shots-per-minute over match. Look for degrading cycle times (overheating?) or stalls.
4. **Match Summary**: Battery voltage + cumulative shots/cycles/peak current/issue count + active hub shots + hub utilization % with HubActive overlay. Spot brownout correlation with scoring dropoff and verify hub window efficiency.
5. **Driver Inputs**: Joystick visualization. Verify driver was commanding what they intended.

**What to look for:**
- Gaps in ReadyToShoot → check which sub-condition failed (AtSpeed? VisionLocked? JamDetected?)
- HeadingErrorRad spikes during shots → robot not aimed at compensated target, check prediction tuning
- ShotPredictor/Active stripe absent when moving → ShootOnTheMove command not running
- Cycle time increasing → check motor temps or battery sag
- Shots stopping → check jam detection or shooter stall

---

### cycle_and_strategy.json: "Are we scoring efficiently?"
**When:** Analyzing scoring strategy, cycle times, and hub shift awareness.
**Tabs:**
1. **Hub Strategy**: HubShiftNumber + TimeToNextShift + TotalShots with HubActive/ReadyToShoot/WonAuto boolean overlays. See exactly when hub windows open and how many shots land in each window.
2. **Cycle Timing**: Last cycle time vs average + shots per minute + completed cycles. Spot degradation trends.
3. **Intake-to-Score Pipeline**: Full mechanism timeline: intake pivot position, shooter velocity, shot count with boolean stripes for IsDeployed, Running, AtSpeed, ShotDetected, JamDetected. Visualize the complete intake→index→shoot pipeline as a time-aligned sequence.
4. **Hub Effectiveness**: Per-shift shot counts (Shift 1-4) on left axis + ActiveHubShots/InactiveHubShots/HubUtilization on right axis with HubActive boolean overlay. Answers "how many shots per hub window?" and "are we wasting shots on inactive hubs?"
5. **Scoring Positions**: 3D field view. See where on the field the robot was scoring from and its path between scoring positions.

**What to look for:**
- Shots during inactive hub windows → wasted effort, improve hub awareness (Tab 4: InactiveHubShots should stay 0)
- Hub utilization low → robot not at-speed during active windows, wasting scoring opportunity
- Per-shift shot imbalance → inconsistent performance across hub windows
- Long gaps between cycles → identify bottleneck (intake? drive? aim?)
- Pipeline stalls → which stage breaks the chain?

---

### pit_triage.json: "Is the robot match-ready?"
**When:** Between matches in the pit. Quick 30-second health check before queueing.
**Tabs:**
1. **Quick Health**: Battery voltage + CAN utilization + loop time with brownout/AllDevicesConnected/enabled boolean overlays. One glance shows if the robot is healthy.
2. **Subsystem Faults**: Active alert count + total issue count with boolean stripes for all fault sources: shooter stall, indexer jam, intake stall, gyro connected, left/right camera connected. Immediately see which subsystem had issues.
3. **Match Performance**: Battery voltage + total shots + health score. Quick performance summary.
4. **Field Replay**: 3D field view. Replay the match to show drive team what happened.

**What to look for:**
- Brownout events → swap battery or reduce current draw
- Disconnected sensors → check camera/gyro wiring before next match
- Low shot count → investigate mechanism issues

---

### mechanism_debug.json: "Why did the shooter/indexer fail?"
**When:** A mechanism misbehaved during the match (missed shots, jams, climb failed).
**Tabs:**
1. **Shooter PID**: Target RPM (red) vs actual (green) vs error (amber) vs output (gray). Classic PID response view.
2. **Shooter Health**: Current draw, temperature, shot detection events, stall detection. Correlate temp spikes with performance drops.
3. **Indexer**: Target vs actual speed, current draw, jam detection. Jams show as current spike + speed drop.
4. **Intake Roller**: Velocity RPM + current draw with Running/Stalled boolean overlays. See intake performance and stall events.
5. **Intake Pivot**: Position (rotations) + current draw with AtTarget/IsDeployed boolean overlays. Verify deploy/retract cycles.
7. **All Motor Temps**: All 4 mechanism motor temps + total system current on one graph. Spot which motor is heating fastest.

**What to look for:**
- Shooter error not converging → PID needs tuning (use shooter_tuning.json for deeper analysis)
- Current spikes without velocity → stall or mechanical jam
- Temperature climbing toward 70C → reduce duty cycle or add cooling time
- Intake stalled → check for obstruction or reduce current limit

---

### power_and_health.json: "Why did we brownout / disconnect / glitch?"
**When:** Robot browned out, had CAN errors, loop overruns, or unexplained behavior.
**Tabs:**
1. **Power Budget**: Battery voltage + brownout state + battery-at-risk prediction + voltage slope. The voltage slope going negative fast while voltage is low = imminent brownout.
2. **CAN Bus**: CAN utilization + TX/RX errors + connected device count with AllDevicesConnected/HasFaults boolean overlays. Error spikes = loose wiring or device dropout. High utilization = too much traffic. Purple line shows connected count (should stay at 8).
3. **Loop Timing**: Loop time in ms + overrun count. Loop time >20ms = missed cycle.
4. **Thermal Profile**: All motor temps + RIO CPU temp on one graph. Identify thermal runaway.
5. **Crash Barriers & Network**: Crash barrier triggers (CommandScheduler, Telemetry, Alerts, Predictive) + bandwidth warning + hardware connectivity (Gyro connected, Left/Right camera connected). Any barrier going true = exception was caught.

**What to look for:**
- Voltage drops correlating with mechanism actions → too much current draw, stagger mechanism starts
- CAN errors spiking → check wiring, terminate bus properly
- Loop time spikes → find what's slow (check LoggedTracer segments in system_overview)
- Crash barriers firing → check exception logs (Diagnostics/Exception/*)

---

## Utility Layouts (focused debugging)

### drive_and_auto.json: "Swerve drive + auto path analysis"
**Tabs:** Field Replay (3D), Speed Profile (measured + requested vX/vY/omega), Path Error (position + heading error + IsFollowing), Gyro Angles (yaw/pitch/roll), Swerve Modules
**When:** Investigating odometry drift, auto path following, swerve module issues, or steering problems. Requested speeds overlay shows command-vs-response lag. Replaces the old separate drive_analysis and auto_debug layouts.

### shooter_tuning.json: Shooter PID tuning session
**Tabs:** Velocity vs Target, At Speed Timeline, Motor Output, Temperature, Shot Detection, Spin-Up Time, At Speed %, Fire Rate, Recovery Time, Shot Prediction, Drift Compensation
**When:** Tuning shooter PID gains and velocity-compensated shooting. Run multiple shots, compare response curves. The Shot Prediction tab shows ComputedRPM vs HeadingErrorRad and distance when ShootOnTheMove is active. The Drift Compensation tab shows DriftX/DriftY aim offsets alongside HeadingSpeedRadPerSec and TimeOfFlightSec.

### system_overview.json: General health check
**Tabs:** Battery, CAN, loop timing, LoggedTracer, temperatures, match state (+ hardware connectivity), alerts, post-match summary
**When:** Quick sanity check that nothing is obviously wrong. Match State tab includes gyro and camera connected overlays.

### vision_debug.json: Vision target tracking
**Tabs:** Target Lock (+ latency, per-camera connected), Consecutive Frames, Ready To Shoot Composite, Target Offset (yaw/pitch)
**When:** Investigating vision dropouts, high latency, camera disconnects, or aim offset issues.

---

## Quick Reference: Which Layout for Which Problem?

| Problem | Open This |
|---------|-----------|
| "Did we score enough?" | match_review |
| "Are our cycles fast?" | cycle_and_strategy → Tab 2 |
| "Hub strategy working?" | cycle_and_strategy → Tab 1 |
| "Per-shift shot breakdown" | cycle_and_strategy → Tab 4 |
| "Wasting shots on inactive hub?" | cycle_and_strategy → Tab 4 |
| "Intake pipeline timing" | cycle_and_strategy → Tab 3 |
| "Quick pit health check" | pit_triage |
| "Auto went sideways" | drive_and_auto → Tab 3 |
| "Swerve drifting" | drive_and_auto → Tab 5 |
| "Shooter wasn't spinning up" | mechanism_debug → Tab 1 |
| "Indexer jammed repeatedly" | mechanism_debug → Tab 3 |
| "Intake stalled" | mechanism_debug → Tab 4 |
| "Climb was slow/failed" | mechanism_debug → Tab 6 |
| "We browned out" | power_and_health → Tab 1 |
| "CAN errors in DS log" | power_and_health → Tab 2 |
| "Device disconnected" | power_and_health → Tab 2 (ConnectedCount drop) |
| "Loop overruns" | power_and_health → Tab 3 |
| "Shooter PID needs tuning" | shooter_tuning |
| "Shot prediction working?" | shooter_tuning → Tab 10 |
| "Drift compensation right?" | shooter_tuning → Tab 11 |
| "Heading off when shooting?" | match_review → Tab 1 (HeadingErrorRad) |
| "Vision kept dropping" | vision_debug |
| "General health check" | system_overview |
