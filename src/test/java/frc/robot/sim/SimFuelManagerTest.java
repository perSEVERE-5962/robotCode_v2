package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Tests for SimFuelManager shot detection and feature flags. */
class SimFuelManagerTest {

    private SimFuelManager manager;
    private FuelPhysicsSim sim;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        FuelPhysicsSim.PhysicsConfig config = new FuelPhysicsSim.PhysicsConfig();
        config.deterministic = true;
        config.deterministicSeed = 42L;
        sim = new FuelPhysicsSim("Test/FuelManager", config);
        manager = new SimFuelManager(sim);
        manager.enable();
    }

    @Test
    void shotDetectionCreatesBall() {
        Pose2d pose = new Pose2d(8, 4, Rotation2d.kZero);
        // Rising edge: false -> true
        manager.update(false, pose, 0, 3000);
        assertEquals(0, sim.getBallCount(), "No ball before rising edge");

        manager.update(true, pose, 0, 3000);
        assertEquals(1, sim.getBallCount(), "Ball should spawn on rising edge");
    }

    @Test
    void noShotOnSustainedHigh() {
        Pose2d pose = new Pose2d(8, 4, Rotation2d.kZero);
        manager.update(true, pose, 0, 3000); // First rising edge
        assertEquals(1, sim.getBallCount());

        manager.update(true, pose, 0, 3000); // Still high, no new edge
        assertEquals(1, sim.getBallCount(), "No new ball without rising edge");
    }

    @Test
    void lowRPMRejectsShotCreation() {
        Pose2d pose = new Pose2d(8, 4, Rotation2d.kZero);
        manager.update(false, pose, 0, 50); // Low RPM
        manager.update(true, pose, 0, 50); // Rising edge but RPM too low
        assertEquals(0, sim.getBallCount(), "Should not spawn ball with RPM <= 100");
    }

    @Test
    void disabledSkipsPhysics() {
        // The enabled TunableNumber defaults to 1, so normally it runs.
        // We test that calling update still works (no crash).
        Pose2d pose = new Pose2d(8, 4, Rotation2d.kZero);
        assertDoesNotThrow(() -> manager.update(false, pose, 0, 3000));
    }

    @Test
    void risingEdgeTrigger() {
        Pose2d pose = new Pose2d(8, 4, Rotation2d.kZero);
        // false -> true -> false -> true should create 2 balls
        manager.update(false, pose, 0, 3000);
        manager.update(true, pose, 0, 3000); // ball 1
        manager.update(false, pose, 0, 3000);
        manager.update(true, pose, 0, 3000); // ball 2

        assertEquals(2, sim.getBallCount(), "Two rising edges should create two balls");
    }

    @Test
    void placeFieldBallsDelegates() {
        manager.placeFieldBalls();
        assertTrue(sim.getBallCount() > 100, "placeFieldBalls should spawn many balls");
    }

    @Test
    void getSim() {
        assertSame(sim, manager.getSim(), "getSim() should return underlying sim");
    }

    @Test
    void startAndStop() {
        manager.disable();
        assertFalse(sim.isRunning());
        manager.enable();
        assertTrue(sim.isRunning());
    }
}
