package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.util.HubShiftEngine.ScheduleConfidence;
import frc.robot.util.HubShiftEngine.ShiftInfo;
import frc.robot.util.HubShiftEngine.ShiftPhase;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class HubShiftEngineTest {

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        NetworkTableInstance.getDefault().startLocal();
        HubShiftEngine.resetInstance();
        FireAuthorization.resetInstance();
        SimHooks.pauseTiming();
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    private void enterTeleop() {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    private void advanceAndUpdate(HubShiftEngine engine, double seconds, double tof) {
        SimHooks.stepTiming(seconds);
        engine.update(tof);
    }

    @Test
    void testInitialStateIsDisabled() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.DISABLED, info.phase());
        assertTrue(info.hubActive(), "Hub should be active when disabled (safe default)");
    }

    @Test
    void testAutoPhaseIsActive() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        engine.update(0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.AUTO, info.phase());
        assertTrue(info.hubActive(), "Both hubs active during auto");
    }

    @Test
    void testUpdateWithoutInitReturnsDisabledEvenInTeleop() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        enterTeleop();
        engine.update(1.0);

        assertEquals(
                ShiftPhase.DISABLED,
                engine.getOfficialInfo().phase(),
                "Without initializeTeleop(), must stay DISABLED even if DS says teleop");
        assertTrue(
                engine.getOfficialInfo().hubActive(),
                "DISABLED state must be hub-active (safe fail-open)");
    }

    @Test
    void testFeatureEnabledDefault() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        assertTrue(engine.isFeatureEnabled(), "Feature should be enabled by default");
        assertTrue(engine.isHubTrackingEnabled(), "Tracking should be enabled by default");
    }

    @Test
    void testGetTeleopElapsed() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        assertEquals(0, engine.getTeleopElapsed(), "Should be 0 before teleop init");
        enterTeleop();
        engine.initializeTeleop();
        SimHooks.stepTiming(5.0);
        assertTrue(engine.getTeleopElapsed() >= 4.5, "Should reflect elapsed time");
    }

    @Test
    void testWonAutoShift1Inactive() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 15.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.SHIFT1, info.phase(), "Should be in SHIFT1 at 15s");
        assertFalse(
                info.hubActive(),
                "Won auto -> SHIFT1 must be INACTIVE (our first inactive window)");
    }

    @Test
    void testWonAutoShift2Active() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 40.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.SHIFT2, info.phase(), "Should be in SHIFT2 at 40s");
        assertTrue(
                info.hubActive(), "Won auto -> SHIFT2 must be ACTIVE (our second active window)");
    }

    @Test
    void testLostAutoShift1Active() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(false);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 15.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.SHIFT1, info.phase());
        assertTrue(
                info.hubActive(),
                "Lost auto -> SHIFT1 must be ACTIVE (compensates for losing auto)");
    }

    @Test
    void testLostAutoShift2Inactive() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(false);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 40.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.SHIFT2, info.phase());
        assertFalse(info.hubActive(), "Lost auto -> SHIFT2 must be INACTIVE");
    }

    @Test
    void testWonScheduleFullPattern() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        boolean[] expected = {true, false, true, false, true, true};
        double[] midpoints = {5.0, 22.0, 47.0, 72.0, 97.0, 125.0};
        ShiftPhase[] phases = {
            ShiftPhase.TRANSITION, ShiftPhase.SHIFT1, ShiftPhase.SHIFT2,
            ShiftPhase.SHIFT3, ShiftPhase.SHIFT4, ShiftPhase.ENDGAME
        };

        for (int i = 0; i < expected.length; i++) {
            HubShiftEngine.resetInstance();
            FireAuthorization.resetInstance();
            engine = HubShiftEngine.getInstance();
            engine.setWonAutoOverride(true);
            enterTeleop();
            engine.initializeTeleop();
            advanceAndUpdate(engine, midpoints[i], 0);

            ShiftInfo info = engine.getOfficialInfo();
            assertEquals(
                    phases[i],
                    info.phase(),
                    "Phase at " + midpoints[i] + "s should be " + phases[i]);
            assertEquals(
                    expected[i],
                    info.hubActive(),
                    "Won schedule: hubActive at " + phases[i] + " should be " + expected[i]);
        }
    }

    @Test
    void testPhaseBoundaryExactTransitionToShift1() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(false);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 10.0, 0);

        assertEquals(
                ShiftPhase.SHIFT1,
                engine.getOfficialInfo().phase(),
                "At exactly 10.0s, should be SHIFT1 (inclusive lower bound)");
    }

    @Test
    void testPhaseBoundaryJustBeforeShift1() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(false);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 9.99, 0);

        assertEquals(
                ShiftPhase.TRANSITION,
                engine.getOfficialInfo().phase(),
                "At 9.99s, should still be in TRANSITION");
    }

    @Test
    void testPastEndgameFallsBackToEndgame() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 145.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(
                ShiftPhase.ENDGAME,
                info.phase(),
                "Past 140s should still report ENDGAME (last phase fallback)");
        assertTrue(info.hubActive(), "ENDGAME is always active for both schedules");
    }

    @Test
    void testConsecutiveMergeRemainingInState() {
        // LOST schedule: TRANSITION + SHIFT1 are both active, so remainingInState merges them
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(false);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 5.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.TRANSITION, info.phase());
        assertTrue(info.hubActive());
        assertEquals(
                30.0,
                info.remainingInState(),
                0.5,
                "LOST schedule: TRANSITION+SHIFT1 are both active, remainingInState merges to"
                    + " SHIFT1_END");
    }

    @Test
    void testTimeUntilDeactivationDuringActivePhase() {
        // WON: TRANSITION(active, 0-10) then SHIFT1(inactive, 10-35)
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 5.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertTrue(info.hubActive());
        assertEquals(
                5.0,
                info.timeUntilDeactivation(),
                0.5,
                "Active phase ending at 10s, elapsed 5s -> 5s until deactivation");
    }

    @Test
    void testTimeToNextActiveDuringInactivePhase() {
        // WON: SHIFT1(inactive, 10-35) then SHIFT2(active, 35-60)
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 20.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertFalse(info.hubActive(), "WON SHIFT1 should be inactive");
        assertEquals(
                15.0,
                info.timeToNextActive(),
                0.5,
                "At 20s in inactive SHIFT1, next active starts at 35s -> 15s to go");
    }

    @Test
    void testShiftedInfoWithZeroTOFFallsBackToOfficial() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 15.0, 0);

        assertEquals(
                engine.getOfficialInfo().hubActive(),
                engine.getShiftedInfo().hubActive(),
                "With TOF=0, shifted should equal official");
        assertEquals(
                engine.getOfficialInfo().phase(),
                engine.getShiftedInfo().phase(),
                "With TOF=0, shifted phase should equal official phase");
    }

    @Test
    void testNegativeTOFClampedToZero() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 15.0, -5.0);

        assertEquals(
                engine.getOfficialInfo().hubActive(),
                engine.getShiftedInfo().hubActive(),
                "Negative TOF clamped to 0 -> no shift");
    }

    @Test
    void testOverrideLocksScheduleThroughMultipleUpdates() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 5.0, 0);
        assertTrue(engine.isWonAuto());
        advanceAndUpdate(engine, 5.0, 0);
        assertTrue(engine.isWonAuto());
        assertEquals(ScheduleConfidence.LOW, engine.getConfidence());
    }

    @Test
    void testGameDataMissingWhenFMSAttached() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        enterTeleop();
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        engine.initializeTeleop();
        engine.update(1.0);

        assertTrue(
                engine.isGameDataMissing(),
                "FMS attached + no game data + no override + in transition -> missing");
    }

    @Test
    void testGameDataNotMissingWithoutFMS() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        enterTeleop();
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        engine.initializeTeleop();
        engine.update(1.0);

        assertFalse(
                engine.isGameDataMissing(),
                "No FMS -> game data missing should be false (this is practice)");
    }

    @Test
    void testGameDataNotMissingWithOverride() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        engine.initializeTeleop();
        engine.update(1.0);

        assertFalse(
                engine.isGameDataMissing(),
                "Copilot override set -> game data missing should be false (deliberate choice)");
    }

    @Test
    void testGameDataNotMissingAfterTransition() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        enterTeleop();
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        engine.initializeTeleop();

        advanceAndUpdate(engine, 11.0, 1.0);

        assertFalse(
                engine.isGameDataMissing(),
                "Past TRANSITION_END (10s), game data missing alert must stop");
    }

    @Test
    void testFMSAttachedNoGameDataIsFallbackConfidence() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        enterTeleop();
        DriverStationSim.setFmsAttached(true);
        DriverStationSim.notifyNewData();
        engine.initializeTeleop();
        engine.update(1.0);

        assertEquals(
                ScheduleConfidence.FALLBACK,
                engine.getConfidence(),
                "FMS attached but no game data -> FALLBACK (50/50 guess)");
    }

    @Test
    void testNonFMSIsLowConfidence() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        enterTeleop();
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        engine.initializeTeleop();
        engine.update(1.0);

        assertEquals(
                ScheduleConfidence.LOW,
                engine.getConfidence(),
                "Non-FMS (practice) -> LOW confidence (SmartDashboard path)");
    }

    @Test
    void testCopilotOverrideSetsLowConfidence() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        assertEquals(ScheduleConfidence.LOW, engine.getConfidence());
        assertTrue(engine.isWonAuto());
    }

    @Test
    void testClearOverrideUnlocks() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        engine.clearWonAutoOverride();
        enterTeleop();
        engine.initializeTeleop();
        engine.update(1.0);
        assertNotEquals(
                ScheduleConfidence.HIGH,
                engine.getConfidence(),
                "Without FMS data, should not be HIGH");
    }

    @Test
    void testDefaultWonAutoIsFalse() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        enterTeleop();
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        engine.initializeTeleop();
        engine.update(1.0);

        assertFalse(
                engine.isWonAuto(), "Default wonAuto should be false (assume lost, conservative)");
    }

    @Test
    void testDisabledTrackingMakesAllPhasesActive() {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        engine.setWonAutoOverride(true);
        enterTeleop();
        engine.initializeTeleop();

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
                "HubShift/TrackingEnabled", 0.0);

        advanceAndUpdate(engine, 15.0, 0);

        ShiftInfo info = engine.getOfficialInfo();
        assertEquals(ShiftPhase.SHIFT1, info.phase(), "Phase detection still works");
        assertTrue(
                info.hubActive(),
                "With hub tracking disabled, SHIFT1 must be active even though WON schedule says"
                    + " inactive");
    }
}
