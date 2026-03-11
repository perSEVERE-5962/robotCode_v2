# Elastic Dashboard Layout Reference

3 Elastic layouts for real-time use. Deployed with robot code.

**JSON files (authoritative):** `robotcode2026-lab/src/main/deploy/elastic/`
**Copies here:** For reference alongside design docs.

---

## competition -- Driver Station (behind the glass)

**File:** `rebuilt_driver_competition.json` (v3.0)
**Design doc:** `driver_competition_design.md`
**Audience:** Driver + operator + coach

### Tab "Match" (10 widgets, 5 rows, no scrolling)

Optimized for the driver. Readable from 5 feet away, scannable in 6 seconds.

| Widget | Type | Size | Signal |
|--------|------|------|--------|
| Match Time | Match Time | 15x1 | Match/Time (color-coded: blue>60s, green>30s, yellow>15s, red) |
| READY TO SHOOT | Boolean Box | 3x3 | Scoring/ReadyToShoot (largest element, green/red) |
| Camera | Camera Stream | 5x3 | USB Camera 0 |
| Target Lock | Boolean Box | 2x2 | Vision/LockedOnTarget (green/orange) |
| Battery | Voltage View | 5x1 | SystemHealth/BatteryVoltage (purpose-built 8-13V) |
| Shooter RPM | Number Bar | 5x1 | Shooter/VelocityRPM (0-5700) |
| Hub Active | Boolean Box | 2x1 | Scoring/Conditions/HubActive |
| JAM! | Boolean Box | 5x1 | Indexer/JamDetected (RED when jammed) |
| Auto Selector | ComboBox Chooser | 5x1 | Auto Chooser |
| Alerts | Text Display | 10x1 | Alerts/ActiveList |

### Tab "Coach" (12 widgets, 4 rows, no scrolling)

Strategy info for the drive coach: shot counts, fire rate, hub timing.

| Widget | Type | Size | Signal |
|--------|------|------|--------|
| Match Time | Match Time | 8x1 | Match/Time |
| READY | Boolean Box | 3x2 | Scoring/ReadyToShoot |
| Battery | Voltage View | 4x1 | SystemHealth/BatteryVoltage |
| Shots Scored | Large Text Display | 3x2 | MatchStats/TotalShots (readable 5+ feet) |
| Fire Rate | Text Display | 2x1 | MatchStats/ShotsPerMinute |
| Hub Active | Boolean Box | 2x1 | Scoring/Conditions/HubActive |
| Shooter RPM | Number Bar | 4x1 | Shooter/VelocityRPM |
| Total Current | Number Bar | 4x1 | SystemHealth/TotalCurrentAmps |
| Hub Shift | Text Display | 2x1 | Scoring/HubShiftNumber |
| JAM! | Boolean Box | 2x1 | Indexer/JamDetected |
| Auto Selector | ComboBox Chooser | 5x1 | Auto Chooser |
| Alerts | Text Display | 10x1 | Alerts/ActiveList |

**v3.0 changes:** Split into 2 tabs (Match + Coach). Match Time upgraded to color-coded Match Time widget. Battery upgraded to Voltage View widget. Added JAM! indicator. Removed Alliance (useless), RP Progress (misleading). Moved Shots/FireRate to Coach tab.

---

## pit -- Pre-Match Diagnostics

**File:** `rebuilt_pit_diagnostic.json` (v3.0)
**Design doc:** `pit_diagnostic_design.md`
**Audience:** Pit crew, viewed close-up between matches

### Tab "Quick Check" (20 widgets, 5 rows, no scrolling)

Go/no-go verdict in 30 seconds. Top = health, middle = motor temps, bottom = sensors + jams.

| Widget | Type | Size | Signal |
|--------|------|------|--------|
| Health Score | Large Text Display | 3x2 | Summary/HealthScore |
| Battery | Voltage View | 5x1 | SystemHealth/BatteryVoltage |
| Top Issue 1 | Text Display | 7x1 | Summary/TopIssue1 |
| Browned Out | Boolean Box | 2x1 | SystemHealth/BrownedOut |
| Total Current | Number Bar | 3x1 | SystemHealth/TotalCurrentAmps |
| Top Issue 2 | Text Display | 4x1 | Summary/TopIssue2 |
| Top Issue 3 | Text Display | 3x1 | Summary/TopIssue3 |
| Shooter Temp | Number Bar | 3x1 | Shooter/TemperatureCelsius |
| Indexer Temp | Number Bar | 3x1 | Indexer/TemperatureCelsius |
| Intake Temp | Number Bar | 3x1 | Intake/TemperatureCelsius |
| IntakeAct Temp | Number Bar | 3x1 | IntakeActuator/TemperatureCelsius |
| Hanger Temp | Number Bar | 3x1 | Hanger/TemperatureCelsius |
| Gyro | Boolean Box | 2x1 | Drive/Gyro/Connected |
| Left Cam | Boolean Box | 2x1 | Vision/Camera/LeftCam/Connected |
| Right Cam | Boolean Box | 2x1 | Vision/Camera/RightCam/Connected |
| Vision Latency | Text Display | 2x1 | Vision/LatencyMs |
| Idx Jam | Boolean Box | 2x1 | Indexer/JamDetected |
| Int Jam | Boolean Box | 2x1 | Intake/JamDetected |
| CAN Util | Number Bar | 3x1 | SystemHealth/CANUtilization |
| Alerts | Text Display | 15x1 | Alerts/ActiveList |

### Tab "Diagnostics" (15 widgets, 4 rows, no scrolling)

Pre-match diagnostic test results. Trigger checks and see pass/fail.

| Widget | Type | Size | Signal |
|--------|------|------|--------|
| Safe Check | Toggle Button | 3x1 | Diagnostics/TriggerSafe |
| Full Check | Toggle Button | 3x1 | Diagnostics/TriggerFull |
| Running | Boolean Box | 2x1 | Diagnostics/Running (blue when active) |
| Mode | Text Display | 2x1 | Diagnostics/Mode |
| Step | Text Display | 5x1 | Diagnostics/Step |
| ALL PASSED | Boolean Box | 5x2 | Diagnostics/AllPassed (big green/red) |
| Pass/Warn/Fail/Skip | 4x Text Display | 2x1 each | Diagnostics/PassCount, WarnCount, FailCount, SkippedCount |
| Gyro Drift | Text Display | 3x1 | Diagnostics/GyroMaxDrift |
| Gyro OK | Boolean Box | 2x1 | Diagnostics/GyroHealthy |
| Vision Drop % | Text Display | 3x1 | Diagnostics/VisionDropoutPct |
| Drive Response | Text Display | 3x1 | Diagnostics/DriveResponseMs |
| Shooter Spinup | Text Display | 3x1 | Diagnostics/ShooterSpinupMs |

### Tab "System Detail" (19 widgets, 4 rows, no scrolling)

Deep dive into bus health, motor currents, and shooter state.

| Widget | Type | Size | Signal |
|--------|------|------|--------|
| Battery | Voltage View | 4x1 | SystemHealth/BatteryVoltage |
| CAN Util | Number Bar | 4x1 | SystemHealth/CANUtilization |
| Loop Time | Number Bar | 4x1 | SystemHealth/LoopTimeMs |
| Bandwidth | Number Bar | 3x1 | Network/BandwidthPercent |
| CAN TX/RX | 2x Text Display | 2x1 each | SystemHealth/CANTxErrors, CANRxErrors |
| CAN Off | Boolean Box | 2x1 | SystemHealth/CANBusOff (red/green) |
| Overruns | Text Display | 2x1 | SystemHealth/LoopOverruns |
| CPU Temp | Number Bar | 3x1 | SystemHealth/RioCPUTemp |
| RSL | Boolean Box | 2x1 | SystemHealth/RSLState |
| BW Warn | Boolean Box | 2x1 | Network/BandwidthCritical |
| Shooter Vel/Tgt | 2x Text Display | 3x1 each | Shooter/VelocityRPM, TargetRPM |
| At Speed | Boolean Box | 2x1 | Shooter/AtSpeed |
| Spinup ms | Text Display | 3x1 | Shooter/SpinUpTimeMs |
| Shots | Text Display | 2x1 | Shooter/TotalShots |
| Shooter/Indexer/Intake Current | 3x Number Bar | 5x1 each | CurrentAmps for each |

**v3.0 changes:** Split 48-widget single-tab layout into 3 tabs. Added Health Score, Top Issues, Voltage View battery. All tabs fit on screen without scrolling.

---

## tuning -- Practice PID Tuning

**File:** `rebuilt_tuning_session.json` (v3.0)
**No design doc** (generic tuning layout)
**Audience:** Programmer during practice sessions

### Tab "Shooter" (26 widgets, 7 rows)

Shooter PID tuning with **Graph widget centerpiece** (8x3 RPM time-series).

| Widget | Type | Size | Signal |
|--------|------|------|--------|
| RPM Graph | **Graph** | 8x3 | Shooter/VelocityRPM (5s window, light blue) |
| kP/kI/kD/FF | 4x Text Display (submit) | 2x1 each | SmartDashboard Shooter gains |
| TargetRPM/Tolerance/ShotDrop | 3x Text Display (submit) | 2-3x1 | SmartDashboard Shooter params |
| TUNING MODE | Boolean Box | 4x2 | TuningMode (green=active, orange=off) |
| AT SPEED | Boolean Box | 4x1 | Shooter/AtSpeed |
| Spin Up / Vel Error | Text Display | 2x1 each | SpinUpTimeMs, VelocityError |
| At Speed % | Number Bar | 3x1 | Shooter/AtSpeedPercent (0-120%) |
| Fire rates + Recovery | 8x Text/Boolean | 2x1 each | Actual/Target fire rate, recovery, vel drop, spinning up, recovery spin up |
| SHOT! | Boolean Box | 1x1 | Shooter/ShotDetected |
| Motor Temp/Current/Output | 3x Number Bar | 5x1 each | Temp, CurrentAmps, AppliedOutput |
| Total Shots | Text Display | 2x1 | Shooter/TotalShots |
| Ready To Shoot | Boolean Box | 4x1 | Scoring/ReadyToShoot |

### Tab "Subsystems" (19 widgets, 5 rows)

Indexer (with current graph), Hanger, Intake, and Driver tuning parameters.

| Widget | Type | Size | Signal |
|--------|------|------|--------|
| Indexer Current Graph | **Graph** | 8x3 | Indexer/CurrentAmps (5s window, orange) |
| Idx kP/kI/kD/FF | 4x Text Display (submit) | 2x1 each | SmartDashboard Indexer gains |
| Idx TgtSpd/JamAmps/JamSec | 3x Text Display (submit) | 2-3x1 | SmartDashboard Indexer params |
| Indexer Jam | Boolean Box | 4x1 | Indexer/JamDetected |
| Idx Actual/Target/Jam Count | 3x Text Display | 3/3/2x1 | Indexer telemetry |
| Hanger kP/kD | 2x Text Display (submit) | 2x1 | SmartDashboard Hanger gains |
| Intake Speed | Text Display (submit) | 2x1 | SmartDashboard Intake speed |
| Vision Yaw/Pitch | 2x Text Display | 3x1 | Vision target angles |
| Deadband / Turn Constant | 2x Text Display (submit) | 3x1 | SmartDashboard Driver params |

**v3.0 changes:** Split into 2 tabs. Added Graph widgets for RPM and indexer current (can't tune PID without time-series). Reorganized tunable parameters next to their graphs.

---

## Elastic Tips

**Creating/editing layouts:** Open Elastic, connect to robot or sim, drag widgets from the widget list to the canvas. Right-click widgets to configure. File > Save Layout.

**ElasticLib notifications:** The robot code can push alerts to the dashboard:
```java
Elastic.sendNotification(new Notification(
    NotificationLevel.WARNING,
    "Brownout Risk",
    "Battery below 11.5V"
));
```
This is more effective than adding widgets -- the alert pops up in the driver's face.

**Tab switching from robot code:**
```java
Elastic.selectTab("Autonomous"); // switch dashboard tab remotely
```

**Competition bandwidth:** Elastic reads from NetworkTables over the field network (4 Mbps limit). Keep published data minimal. Log to USB for AdvantageScope instead.
