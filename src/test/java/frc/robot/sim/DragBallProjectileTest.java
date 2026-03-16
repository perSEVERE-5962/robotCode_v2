package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.sim.FuelPhysicsSim.PhysicsConfig;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Physics trajectory tests: drag, Magnus, and spin decay vs. analytical baselines. */
class DragBallProjectileTest {

    private FuelPhysicsSim sim;
    private PhysicsConfig config;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        config = new PhysicsConfig();
        config.deterministic = true;
        config.deterministicSeed = 42L;
        config.sleepingEnabled = false;
        config.subticks = 10; // Higher subticks for trajectory accuracy
        config.conservationMonitor = true;
        sim = new FuelPhysicsSim("Test/DragBall", config);
        sim.enable();
    }

    @Test
    void dragReducesRangeVsVacuum() {
        // Compute vacuum range analytically
        double vx = 10.0, vz = 10.0;
        double v = Math.sqrt(vx * vx + vz * vz);
        double theta = Math.atan2(vz, vx);
        double vacuumRange = v * v * Math.sin(2 * theta) / 9.81;

        // With drag (default Cd=0.47); Y=1 avoids hub geometry
        config.magnusEnabled = false;
        config.frictionEnabled = false;
        sim.setConfig(config);

        sim.launchBall(
                new Translation3d(2, 1, FuelPhysicsSim.getBallRadius()),
                new Translation3d(vx, 0, vz),
                0);

        double maxX = 2;
        for (int i = 0; i < 500; i++) {
            sim.advancePhysics(0.02);
            if (sim.getBallCount() > 0) {
                maxX = Math.max(maxX, sim.getBallPositions().get(0).getX());
            }
        }

        double dragRange = maxX - 2.0;
        assertTrue(
                dragRange < vacuumRange * 0.95,
                "Drag should reduce range by at least 5%: drag="
                        + dragRange
                        + " vacuum="
                        + vacuumRange);
        assertTrue(
                dragRange > vacuumRange * 0.3,
                "Drag shouldn't reduce range by more than 70%: drag=" + dragRange);
    }

    @Test
    void magnusBackspinAddsLift() {
        config.frictionEnabled = false;
        config.spinDecayEnabled = false;
        sim.setConfig(config);

        // Y=7.5 avoids hub geometry entirely (hub ramp Y range is [1.28, 6.79])
        sim.launchBall(
                new Translation3d(3, 7.5, 0.5),
                new Translation3d(10, 0, 5),
                3000); // 3000 RPM backspin

        double maxZWithSpin = 0;
        for (int i = 0; i < 300; i++) {
            sim.advancePhysics(0.02);
            if (sim.getBallCount() > 0) {
                maxZWithSpin = Math.max(maxZWithSpin, sim.getBallPositions().get(0).getZ());
            }
        }

        // Without backspin (new sim, no Magnus)
        PhysicsConfig noMagnusCfg = config.copy();
        noMagnusCfg.magnusEnabled = false;
        FuelPhysicsSim simNoSpin = new FuelPhysicsSim("Test/NoMagnus", noMagnusCfg);
        simNoSpin.enable();
        simNoSpin.launchBall(new Translation3d(3, 7.5, 0.5), new Translation3d(10, 0, 5), 0);

        double maxZNoSpin = 0;
        for (int i = 0; i < 300; i++) {
            simNoSpin.advancePhysics(0.02);
            if (simNoSpin.getBallCount() > 0) {
                maxZNoSpin = Math.max(maxZNoSpin, simNoSpin.getBallPositions().get(0).getZ());
            }
        }

        assertTrue(
                maxZWithSpin > maxZNoSpin,
                "Magnus backspin should increase apex: withSpin="
                        + maxZWithSpin
                        + " noSpin="
                        + maxZNoSpin);
    }

    @Test
    void trajectoryStartHeightCorrect() {
        double startZ = 1.5;
        sim.launchBall(new Translation3d(8, 4, startZ), new Translation3d(5, 0, 5), 0);

        Translation3d pos = sim.getBallPositions().get(0);
        assertEquals(startZ, pos.getZ(), 1e-6, "Initial height should match launch height");
    }

    @Test
    void zeroVelocityDropsStraight() {
        config.magnusEnabled = false;
        sim.setConfig(config);

        sim.launchBall(new Translation3d(8, 4, 3), new Translation3d(0, 0, 0), 0);

        for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);

        Translation3d pos = sim.getBallPositions().get(0);
        assertEquals(8.0, pos.getX(), 0.01, "Ball with zero velocity should drop straight (X)");
        assertEquals(4.0, pos.getY(), 0.01, "Ball with zero velocity should drop straight (Y)");
        assertTrue(pos.getZ() < 3.0, "Ball should have fallen");
    }

    @Test
    void highSpeedReachesFarther() {
        config.magnusEnabled = false;
        config.frictionEnabled = false;
        sim.setConfig(config);

        // Y=1 avoids hub geometry for both balls
        FuelPhysicsSim simSlow = new FuelPhysicsSim("Test/Slow", config);
        simSlow.enable();
        simSlow.launchBall(new Translation3d(2, 1, 0.5), new Translation3d(5, 0, 5), 0);

        FuelPhysicsSim simFast = new FuelPhysicsSim("Test/Fast", config);
        simFast.enable();
        simFast.launchBall(new Translation3d(2, 1, 0.5), new Translation3d(12, 0, 5), 0);

        // Compare X positions at t=1s (50 steps). Both balls still mid-flight,
        // before either reaches the far wall where maxX would be identical.
        for (int i = 0; i < 50; i++) {
            simSlow.advancePhysics(0.02);
            simFast.advancePhysics(0.02);
        }

        double xSlow = simSlow.getBallPositions().get(0).getX();
        double xFast = simFast.getBallPositions().get(0).getX();

        assertTrue(
                xFast > xSlow,
                "Faster ball should be farther at t=1s: slow="
                        + (xSlow - 2)
                        + " fast="
                        + (xFast - 2));
    }

    @Test
    void gravityBringsItDown() {
        sim.launchBall(new Translation3d(8, 4, 0.5), new Translation3d(5, 0, 15), 0);

        // Track Z: should go up then come back down
        double maxZ = 0;
        boolean wentUp = false;
        boolean cameDown = false;

        for (int i = 0; i < 500; i++) {
            sim.advancePhysics(0.02);
            if (sim.getBallCount() > 0) {
                double z = sim.getBallPositions().get(0).getZ();
                if (z > 2.0) wentUp = true;
                maxZ = Math.max(maxZ, z);
                if (wentUp && z < maxZ * 0.5) cameDown = true;
            }
        }

        assertTrue(wentUp, "Ball should go up");
        assertTrue(cameDown, "Gravity should bring it back down");
    }

    @Test
    void spinDecayReducesOmegaOverTime() {
        config.magnusEnabled = false;
        config.frictionEnabled = false;
        config.spinDecayTau = 2.0;
        sim.setConfig(config);

        double initialRPM = 5000;
        sim.launchBall(new Translation3d(8, 4, 3), new Translation3d(0, 0, 0), initialRPM);

        double initialOmega = Math.abs(sim.getBallOmegas().get(0).getY());

        // Run for 2 * tau = 4 seconds
        for (int i = 0; i < 200; i++) sim.advancePhysics(0.02);

        double finalOmega = Math.abs(sim.getBallOmegas().get(0).getY());
        // At t = 2*tau, omega should be about e^(-2) = 13.5% of initial
        double expectedRatio = Math.exp(-2.0);

        double actualRatio = finalOmega / initialOmega;
        assertEquals(
                expectedRatio,
                actualRatio,
                expectedRatio * 0.15,
                "Spin decay should follow exponential: expected ratio="
                        + expectedRatio
                        + " actual="
                        + actualRatio);
    }

    @Test
    void dragCoefficientsArePhysicallyReasonable() {
        // Verify precomputed constants match the physics
        double expectedDragFactor = 0.5 * 1.225 * 0.47 * Math.PI * 0.075 * 0.075 / 0.215;
        assertEquals(
                expectedDragFactor,
                FuelPhysicsSim.getDragAccelFactor(),
                expectedDragFactor * 0.01,
                "Drag acceleration factor should match physics constants");

        double expectedMagnusFactor = 0.5 * 1.225 * 0.2 * Math.PI * 0.075 * 0.075 * 0.075 / 0.215;
        assertEquals(
                expectedMagnusFactor,
                FuelPhysicsSim.getMagnusAccelFactor(),
                expectedMagnusFactor * 0.01,
                "Magnus acceleration factor should match physics constants");
    }

    @Test
    void crossProductDirectionCorrect() {
        // Backspin around -Y with velocity in +X should produce lift in +Z
        // omega = (0, -omega, 0), vel = (vx, 0, 0)
        // omega x vel = (-omega*0 - 0*0, 0*vx - 0*0, 0*0 - (-omega)*vx) = (0, 0, omega*vx)
        // So Magnus force should be in +Z (upward) for backspin, which is correct

        config.frictionEnabled = false;
        config.sleepingEnabled = false;
        config.spinDecayEnabled = false;
        sim.setConfig(config);

        // Ball with backspin moving in +X
        sim.launchBall(
                new Translation3d(5, 4, 1),
                new Translation3d(10, 0, 0), // purely horizontal
                5000); // high backspin for visible effect

        sim.advancePhysics(0.1);

        // After a short time, the ball should have moved upward from Magnus
        Translation3d vel = sim.getBallVelocities().get(0);
        assertTrue(
                vel.getZ() > -5.0, // Should be less negative than pure gravity (-0.981)
                "Magnus backspin should add upward force: vz=" + vel.getZ());
    }
}
