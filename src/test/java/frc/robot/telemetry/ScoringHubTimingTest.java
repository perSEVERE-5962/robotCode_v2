package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.util.FireAuthorization;
import frc.robot.util.HubShiftEngine;
import frc.robot.util.ScoringReadiness;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests that ScoringReadiness correctly reads hub shift state from HubShiftEngine. The detailed
 * phase/boundary logic is covered in HubShiftEngineTest; these tests verify the delegation works.
 */
class ScoringHubTimingTest {

    @BeforeEach
    void setUp() throws Exception {
        HAL.initialize(500, 0);
        NetworkTableInstance.getDefault().startLocal();
        // resetInstance() is package-private, use reflection from this package
        java.lang.reflect.Field hubField = HubShiftEngine.class.getDeclaredField("instance");
        hubField.setAccessible(true);
        hubField.set(null, null);
        java.lang.reflect.Field faField = FireAuthorization.class.getDeclaredField("instance");
        faField.setAccessible(true);
        faField.set(null, null);
        ScoringReadiness.getInstance().reset();
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

    private void advanceAndUpdate(double seconds) {
        HubShiftEngine engine = HubShiftEngine.getInstance();
        SimHooks.stepTiming(seconds);
        engine.update(0);
        ScoringReadiness.getInstance().update();
    }

    @Test
    void testShiftNumber1() {
        HubShiftEngine.getInstance().setWonAutoOverride(false);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(15.0); // SHIFT1: 10-35s

        assertEquals(1, ScoringReadiness.getInstance().getHubShiftNumber());
    }

    @Test
    void testShiftNumber4() {
        HubShiftEngine.getInstance().setWonAutoOverride(false);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(97.0); // SHIFT4: 85-110s

        assertEquals(4, ScoringReadiness.getInstance().getHubShiftNumber());
    }

    @Test
    void testShiftNumberEndgame() {
        HubShiftEngine.getInstance().setWonAutoOverride(false);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(120.0); // ENDGAME: 110-140s

        assertEquals(0, ScoringReadiness.getInstance().getHubShiftNumber());
    }

    @Test
    void testTransitionAlwaysActive() {
        HubShiftEngine.getInstance().setWonAutoOverride(true);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(5.0); // TRANSITION: 0-10s

        assertTrue(ScoringReadiness.getInstance().isHubActive());
    }

    @Test
    void testEndgameAlwaysActive() {
        HubShiftEngine.getInstance().setWonAutoOverride(true);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(125.0);

        assertTrue(ScoringReadiness.getInstance().isHubActive());
    }

    @Test
    void testShift1WonAuto() {
        HubShiftEngine.getInstance().setWonAutoOverride(true);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(15.0);

        assertFalse(
                ScoringReadiness.getInstance().isHubActive(),
                "Winner should be INACTIVE during odd shift 1");
    }

    @Test
    void testShift1LostAuto() {
        HubShiftEngine.getInstance().setWonAutoOverride(false);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(15.0);

        assertTrue(
                ScoringReadiness.getInstance().isHubActive(),
                "Loser should be ACTIVE during odd shift 1");
    }

    @Test
    void testShift2WonAuto() {
        HubShiftEngine.getInstance().setWonAutoOverride(true);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(40.0);

        assertTrue(
                ScoringReadiness.getInstance().isHubActive(),
                "Winner should be ACTIVE during even shift 2");
    }

    @Test
    void testTimeToNextShift() {
        HubShiftEngine.getInstance().setWonAutoOverride(true);
        enterTeleop();
        HubShiftEngine.getInstance().initializeTeleop();

        advanceAndUpdate(5.0); // TRANSITION at 5s, 5s remaining

        double time = ScoringReadiness.getInstance().getTimeToNextShiftSec();
        assertTrue(time > 0, "Should have positive time remaining");
    }
}
