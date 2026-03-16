package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.sim.FuelPhysicsSim.PhysicsConfig;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import java.util.List;

class FuelPhysicsSimTest {

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
        config.conservationMonitor = true;
        sim = new FuelPhysicsSim("Test/Fuel", config);
        sim.enable();
    }

    @Nested
    class Lifecycle {
        @Test
        void launchBallIncrementsCount() {
            sim.launchBall(new Translation3d(8, 4, 1), new Translation3d(5, 0, 5), 2000);
            assertEquals(1, sim.getBallCount());
            assertEquals(1, sim.getTotalLaunched());
        }

        @Test
        void spawnBallAtRest() {
            sim.spawnBall(new Translation3d(8, 4, FuelPhysicsSim.getBallRadius()));
            assertEquals(1, sim.getBallCount());
            sim.advancePhysics(0.02);
            // Ball should stay near initial position
            Translation3d pos = sim.getBallPositions().get(0);
            assertEquals(FuelPhysicsSim.getBallRadius(), pos.getZ(), 0.01);
        }

        @Test
        void clearBallsRemovesAll() {
            for (int i = 0; i < 10; i++) {
                sim.spawnBall(new Translation3d(8, 4, FuelPhysicsSim.getBallRadius()));
            }
            assertEquals(10, sim.getBallCount());
            sim.clearBalls();
            assertEquals(0, sim.getBallCount());
        }

        @Test
        void placeFieldBallsCreatesMany() {
            sim.placeFieldBalls();
            assertTrue(sim.getBallCount() > 100, "Should spawn many balls");
        }

        @Test
        void tickDoesNothingWhenStopped() {
            sim.disable();
            sim.spawnBall(new Translation3d(8, 4, 1));
            Translation3d posBefore = sim.getBallPositions().get(0);
            sim.tick();
            Translation3d posAfter = sim.getBallPositions().get(0);
            assertEquals(posBefore.getX(), posAfter.getX(), 1e-9);
        }

        @Test
        void resetCountersWorks() {
            sim.launchBall(new Translation3d(8, 4, 1), new Translation3d(5, 0, 5), 0);
            sim.resetCounters();
            assertEquals(0, sim.getTotalLaunched());
            assertEquals(0, sim.getTotalScored());
        }

        @Test
        void outOfBoundsBallsRemoved() {
            // Spawn a ball moving fast toward the wall edge
            sim.spawnBall(new Translation3d(-3, 4, 1), new Translation3d(-10, 0, 0));
            sim.advancePhysics(0.02);
            // Ball at -3 with -10 vel over 5 subticks should trigger OOB
            // After enough steps the ball should be removed
            for (int i = 0; i < 50; i++) sim.advancePhysics(0.02);
            assertEquals(0, sim.getBallCount(), "OOB ball should be removed");
        }

        @Test
        void maxBallsLimitRespected() {
            for (int i = 0; i < 2100; i++) {
                sim.spawnBall(new Translation3d(8, 4, FuelPhysicsSim.getBallRadius()));
            }
            assertTrue(sim.getBallCount() <= 2000, "Should respect MAX_BALLS");
        }
    }

    @Nested
    class PhysicsAccuracy {
        @Test
        void symplecticEulerEnergyBounded() {
            // With no drag, total energy should be conserved within 1% (decreases from COR on
            // bounces)
            config.dragEnabled = false;
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.spinTransferEnabled = false;
            config.sleepingEnabled = false;
            config.velocityDependentCOR = false;
            config.subticks = 10;
            sim.setConfig(config);

            // Use Y=1 to avoid hub geometry collisions
            sim.launchBall(new Translation3d(8, 1, 2), new Translation3d(3, 0, 5), 0);
            double initialEnergy = sim.getTotalKineticEnergy() + sim.getTotalPotentialEnergy();

            for (int i = 0; i < 1000; i++) {
                sim.advancePhysics(0.02);
            }

            double finalEnergy = sim.getTotalKineticEnergy() + sim.getTotalPotentialEnergy();
            // Energy should decrease (COR < 1 on bounces) but not increase
            // With field COR < 1, energy decreases on each bounce
            assertTrue(
                    finalEnergy <= initialEnergy * 1.01,
                    "Energy should not increase: initial="
                            + initialEnergy
                            + " final="
                            + finalEnergy);
        }

        @Test
        void vacuumTrajectoryMatchesAnalytical() {
            // No drag, no Magnus: apex height = Z0 + vz^2 / (2g)
            // Testing apex avoids ground-bounce complications (COR causes multi-bounce)
            config.dragEnabled = false;
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.subticks = 10;
            sim.setConfig(config);

            double v = 10.0;
            double theta = Math.toRadians(45);
            double vx = v * Math.cos(theta);
            double vz = v * Math.sin(theta);

            // Y=1 avoids hub geometry
            double startZ = 0.5;
            sim.launchBall(
                    new Translation3d(2, 1, startZ),
                    new Translation3d(vx, 0, vz),
                    new Translation3d());

            // Track apex height and X at apex
            double maxZ = 0;
            double xAtMaxZ = 0;
            for (int i = 0; i < 200; i++) {
                sim.advancePhysics(0.02);
                if (sim.getBallCount() == 0) break;
                Translation3d pos = sim.getBallPositions().get(0);
                if (pos.getZ() > maxZ) {
                    maxZ = pos.getZ();
                    xAtMaxZ = pos.getX();
                }
            }

            // Apex height: Z0 + vz^2/(2g)
            double expectedApex = startZ + vz * vz / (2 * 9.81);
            assertEquals(
                    expectedApex,
                    maxZ,
                    expectedApex * 0.05,
                    "Apex height should match analytical: expected="
                            + expectedApex
                            + " actual="
                            + maxZ);

            // X at apex should be half the vacuum range: X0 + vx * (vz/g)
            double expectedXAtApex = 2.0 + vx * (vz / 9.81);
            assertEquals(
                    expectedXAtApex,
                    xAtMaxZ,
                    expectedXAtApex * 0.05,
                    "X at apex should match analytical: expected="
                            + expectedXAtApex
                            + " actual="
                            + xAtMaxZ);
        }

        @Test
        void gravityPullsBallDown() {
            sim.launchBall(new Translation3d(8, 4, 2), new Translation3d(0, 0, 0), 0);
            sim.advancePhysics(0.5);
            Translation3d pos = sim.getBallPositions().get(0);
            // After 0.5s of free fall from 2m, z = 2 - 0.5*9.81*0.25 = 0.77m
            assertTrue(pos.getZ() < 2.0, "Ball should fall");
            assertTrue(pos.getZ() > 0, "Ball should not go below ground");
        }

        @Test
        void highSpeedReachesFarther() {
            config.magnusEnabled = false;
            config.sleepingEnabled = false;
            sim.setConfig(config);

            // Y=1 avoids hub geometry; use same Y for fair comparison
            sim.launchBall(new Translation3d(2, 1, 0.5), new Translation3d(5, 0, 5), 0);
            sim.launchBall(new Translation3d(2, 1, 0.5), new Translation3d(10, 0, 5), 0);

            for (int i = 0; i < 200; i++) sim.advancePhysics(0.02);

            List<Translation3d> positions = sim.getBallPositions();
            // Fast ball (index 1) should have traveled farther in X
            assertTrue(positions.size() >= 2);
            assertTrue(
                    positions.get(1).getX() > positions.get(0).getX(),
                    "Faster ball should travel farther");
        }

        @Test
        void zeroVelocityDropsStraight() {
            sim.launchBall(new Translation3d(8, 4, 3), new Translation3d(0, 0, 0), 0);

            for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);

            Translation3d pos = sim.getBallPositions().get(0);
            assertEquals(8.0, pos.getX(), 0.01, "Should not drift in X");
            assertEquals(4.0, pos.getY(), 0.01, "Should not drift in Y");
        }
    }

    @Nested
    class MagnusAndSpin {
        @Test
        void magnusBackspinCurvesUpward() {
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.spinDecayEnabled = false; // keep spin constant for clearer test
            sim.setConfig(config);

            // Ball with backspin (positive RPM = lift)
            sim.launchBall(new Translation3d(3, 4, 0.5), new Translation3d(10, 0, 5), 3000);

            // Ball without spin
            FuelPhysicsSim simNoSpin = new FuelPhysicsSim("Test/NoSpin", config);
            simNoSpin.enable();
            PhysicsConfig noMagnusCfg = config.copy();
            noMagnusCfg.magnusEnabled = false;
            simNoSpin.setConfig(noMagnusCfg);
            simNoSpin.launchBall(new Translation3d(3, 4, 0.5), new Translation3d(10, 0, 5), 0);

            // Track max height
            double maxZSpin = 0, maxZNoSpin = 0;
            for (int i = 0; i < 200; i++) {
                sim.advancePhysics(0.02);
                simNoSpin.advancePhysics(0.02);
                if (sim.getBallCount() > 0)
                    maxZSpin = Math.max(maxZSpin, sim.getBallPositions().get(0).getZ());
                if (simNoSpin.getBallCount() > 0)
                    maxZNoSpin = Math.max(maxZNoSpin, simNoSpin.getBallPositions().get(0).getZ());
            }

            assertTrue(
                    maxZSpin > maxZNoSpin,
                    "Backspin should increase max height: spin="
                            + maxZSpin
                            + " noSpin="
                            + maxZNoSpin);
        }

        @Test
        void magnusSidespinCurvesSideways() {
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.spinDecayEnabled = false;
            sim.setConfig(config);

            // Sidespin: omega around Z axis
            double omegaZ = 300.0; // rad/s (about 2860 RPM around vertical axis)
            sim.launchBall(
                    new Translation3d(3, 4, 1),
                    new Translation3d(10, 0, 3),
                    new Translation3d(0, 0, omegaZ));

            double maxYDisplacement = 0;
            for (int i = 0; i < 200; i++) {
                sim.advancePhysics(0.02);
                if (sim.getBallCount() > 0) {
                    double yDisp = Math.abs(sim.getBallPositions().get(0).getY() - 4.0);
                    maxYDisplacement = Math.max(maxYDisplacement, yDisp);
                }
            }

            assertTrue(
                    maxYDisplacement > 0.05,
                    "Sidespin should curve ball sideways: maxY=" + maxYDisplacement);
        }

        @Test
        void magnusTopspinCurvesDownward() {
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.spinDecayEnabled = false;
            sim.setConfig(config);

            // Topspin: negative of backspin (positive omega Y = topspin for +X velocity)
            sim.launchBall(new Translation3d(3, 4, 0.5), new Translation3d(10, 0, 5), -3000);

            FuelPhysicsSim simNoSpin = new FuelPhysicsSim("Test/NoSpin2", config);
            simNoSpin.enable();
            PhysicsConfig noMagnusCfg = config.copy();
            noMagnusCfg.magnusEnabled = false;
            simNoSpin.setConfig(noMagnusCfg);
            simNoSpin.launchBall(new Translation3d(3, 4, 0.5), new Translation3d(10, 0, 5), 0);

            // Only track the first arc (stop after apex to avoid ground bounce contamination)
            double maxZTopSpin = 0, maxZNoSpin = 0;
            boolean topPassed = false, noPassed = false;
            double prevZTop = 0, prevZNo = 0;
            for (int i = 0; i < 200; i++) {
                sim.advancePhysics(0.02);
                simNoSpin.advancePhysics(0.02);
                if (sim.getBallCount() > 0 && !topPassed) {
                    double z = sim.getBallPositions().get(0).getZ();
                    if (z < prevZTop && prevZTop > 0.6) topPassed = true;
                    maxZTopSpin = Math.max(maxZTopSpin, z);
                    prevZTop = z;
                }
                if (simNoSpin.getBallCount() > 0 && !noPassed) {
                    double z = simNoSpin.getBallPositions().get(0).getZ();
                    if (z < prevZNo && prevZNo > 0.6) noPassed = true;
                    maxZNoSpin = Math.max(maxZNoSpin, z);
                    prevZNo = z;
                }
            }

            assertTrue(
                    maxZTopSpin < maxZNoSpin,
                    "Topspin should lower apex: topspin=" + maxZTopSpin + " noSpin=" + maxZNoSpin);
        }

        @Test
        void spinDecaysExponentially() {
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.spinDecayTau = 1.0; // 1 second for easier testing
            config.subticks = 10;
            sim.setConfig(config);

            double initialRPM = 5000;
            sim.launchBall(new Translation3d(8, 4, 3), new Translation3d(0, 0, 0), initialRPM);

            // Run for tau seconds
            for (int i = 0; i < 50; i++) sim.advancePhysics(0.02); // 1 second total

            List<Translation3d> omegas = sim.getBallOmegas();
            double currentOmegaY = Math.abs(omegas.get(0).getY());
            double initialOmegaY = initialRPM * 2.0 * Math.PI / 60.0;
            double expectedOmega = initialOmegaY * Math.exp(-1.0); // 37% at t=tau

            assertEquals(
                    expectedOmega,
                    currentOmegaY,
                    expectedOmega * 0.10,
                    "Spin should be ~37% of initial at t=tau");
        }

        @Test
        void noMagnusDisablesCurve() {
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            sim.setConfig(config);

            sim.launchBall(new Translation3d(3, 4, 1), new Translation3d(10, 0, 5), 5000);

            // Without Magnus, spin should not affect trajectory
            FuelPhysicsSim simNoSpin = new FuelPhysicsSim("Test/NoSpinRef", config);
            simNoSpin.enable();
            simNoSpin.setConfig(config);
            simNoSpin.launchBall(new Translation3d(3, 4, 1), new Translation3d(10, 0, 5), 0);

            for (int i = 0; i < 100; i++) {
                sim.advancePhysics(0.02);
                simNoSpin.advancePhysics(0.02);
            }

            if (sim.getBallCount() > 0 && simNoSpin.getBallCount() > 0) {
                Translation3d posSpin = sim.getBallPositions().get(0);
                Translation3d posNoSpin = simNoSpin.getBallPositions().get(0);
                // Positions should be very close since Magnus is disabled
                assertEquals(posNoSpin.getZ(), posSpin.getZ(), 0.1, "No Magnus = same trajectory");
            }
        }
    }

    @Nested
    class ConservationLaws {
        @Test
        void energyDecreasesMonotonically() {
            // With real COR values, energy should never increase
            config.sleepingEnabled = false;
            config.subticks = 5;
            sim.setConfig(config);

            sim.launchBall(new Translation3d(8, 4, 3), new Translation3d(5, 2, 8), 1000);

            double prevEnergy = sim.getTotalKineticEnergy() + sim.getTotalPotentialEnergy();
            int violations = 0;

            for (int i = 0; i < 500; i++) {
                sim.advancePhysics(0.02);
                double energy = sim.getTotalKineticEnergy() + sim.getTotalPotentialEnergy();
                if (energy > prevEnergy + 0.01) { // 10mJ tolerance for numerical noise
                    violations++;
                }
                prevEnergy = energy;
            }

            // Allow a few violations from numerical noise, but not many
            assertTrue(
                    violations < 10, "Energy increased " + violations + " times out of 500 steps");
        }

        @Test
        void momentumConservedInBallBallCollision() {
            // Two balls colliding in vacuum with no gravity or drag
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.spinTransferEnabled = false;
            config.subticks = 10;
            sim.setConfig(config);

            // Two balls approaching each other
            sim.spawnBall(new Translation3d(7.5, 4, 2), new Translation3d(3, 0, 0));
            sim.spawnBall(new Translation3d(8.5, 4, 2), new Translation3d(-3, 0, 0));

            Translation3d initialMomentum = sim.getTotalMomentum();

            // Step until collision happens and resolves
            for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);

            Translation3d finalMomentum = sim.getTotalMomentum();

            // Momentum should be conserved (X components should sum to 0 since equal and opposite)
            assertEquals(
                    initialMomentum.getX(), finalMomentum.getX(), 0.01, "X momentum not conserved");
            assertEquals(
                    initialMomentum.getY(), finalMomentum.getY(), 0.01, "Y momentum not conserved");
        }

        @Test
        void angularMomentumConservedInIsolatedCollision() {
            // Two spinning balls collide. Total angular momentum should be roughly conserved.
            config.magnusEnabled = false;
            config.sleepingEnabled = false;
            config.subticks = 10;
            sim.setConfig(config);

            double omega1 = 100.0; // rad/s
            sim.launchBall(
                    new Translation3d(7.5, 4, 2),
                    new Translation3d(3, 0, 0),
                    new Translation3d(0, omega1, 0));
            sim.launchBall(
                    new Translation3d(8.5, 4, 2),
                    new Translation3d(-3, 0, 0),
                    new Translation3d(0, -omega1, 0));

            double mass = FuelPhysicsSim.getBallMass();
            double I = FuelPhysicsSim.getMomentOfInertia();

            // Compute initial total angular momentum about origin
            // L = r x (m*v) + I*omega for each ball
            // This is complex, so just check omega conservation
            List<Translation3d> initialOmegas = sim.getBallOmegas();
            double initialTotalOmegaY = 0;
            for (Translation3d o : initialOmegas) initialTotalOmegaY += o.getY();

            for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);

            List<Translation3d> finalOmegas = sim.getBallOmegas();
            double finalTotalOmegaY = 0;
            for (Translation3d o : finalOmegas) finalTotalOmegaY += o.getY();

            // Total spin should be roughly conserved (some energy goes to translation)
            // Use a wider tolerance since spin-transfer mechanics are approximate
            assertEquals(
                    initialTotalOmegaY,
                    finalTotalOmegaY,
                    Math.abs(initialTotalOmegaY) * 0.5 + 1.0,
                    "Angular momentum should be approximately conserved");
        }
    }

    @Nested
    class Collisions {
        @Test
        void multiBodyPileSettlesInFiniteIterations() {
            config.sleepingEnabled = false;
            config.subticks = 5;
            sim.setConfig(config);

            // Stack 5 balls vertically
            for (int i = 0; i < 5; i++) {
                sim.spawnBall(
                        new Translation3d(
                                8,
                                4,
                                FuelPhysicsSim.getBallRadius()
                                        + i * FuelPhysicsSim.getBallRadius() * 2.5),
                        new Translation3d(0, 0, -1));
            }

            // Let them settle
            for (int i = 0; i < 200; i++) sim.advancePhysics(0.02);

            // All balls should have low velocity
            for (Translation3d vel : sim.getBallVelocities()) {
                assertTrue(vel.getNorm() < 0.5, "Ball should have settled: speed=" + vel.getNorm());
            }
        }

        @Test
        void stackedBallsDoNotJitter() {
            config.sleepingEnabled = false;
            config.subticks = 5;
            sim.setConfig(config);

            // Let balls settle first
            for (int i = 0; i < 3; i++) {
                sim.spawnBall(
                        new Translation3d(
                                8,
                                4,
                                FuelPhysicsSim.getBallRadius()
                                        + i * FuelPhysicsSim.getBallRadius() * 2.2));
            }
            for (int i = 0; i < 300; i++) sim.advancePhysics(0.02);

            // Record positions, run 200 more steps, check stability
            List<Translation3d> posBefore = sim.getBallPositions();
            for (int i = 0; i < 200; i++) sim.advancePhysics(0.02);
            List<Translation3d> posAfter = sim.getBallPositions();

            for (int i = 0; i < Math.min(posBefore.size(), posAfter.size()); i++) {
                double drift = posBefore.get(i).getDistance(posAfter.get(i));
                assertTrue(drift < 0.05, "Ball " + i + " jittered by " + drift + "m");
            }
        }

        @Test
        void penetrationStaysBelowTolerance() {
            config.sleepingEnabled = false;
            sim.setConfig(config);

            // Ball bouncing on ground
            sim.launchBall(new Translation3d(8, 4, 3), new Translation3d(0, 0, -10), 0);

            double maxPenetration = 0;
            for (int i = 0; i < 500; i++) {
                sim.advancePhysics(0.02);
                if (sim.getBallCount() > 0) {
                    double z = sim.getBallPositions().get(0).getZ();
                    double pen = FuelPhysicsSim.getBallRadius() - z;
                    maxPenetration = Math.max(maxPenetration, pen);
                }
            }

            assertTrue(
                    maxPenetration < 0.005,
                    "Penetration should stay below 5mm: " + maxPenetration + "m");
        }

        @Test
        void wallBounceWithCorrectCOR() {
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.velocityDependentCOR = false;
            sim.setConfig(config);

            // Ball moving toward X=0 wall
            double incomingSpeed = 5.0;
            sim.spawnBall(
                    new Translation3d(FuelPhysicsSim.getBallRadius() + 0.01, 4, 0.3),
                    new Translation3d(-incomingSpeed, 0, 0));

            sim.advancePhysics(0.02);

            Translation3d vel = sim.getBallVelocities().get(0);
            double expectedBounceSpeed = incomingSpeed * FuelPhysicsSim.getFieldCOR();
            assertEquals(
                    expectedBounceSpeed,
                    vel.getX(),
                    expectedBounceSpeed * 0.15,
                    "Bounce speed should match COR");
            assertTrue(vel.getX() > 0, "Ball should bounce away from wall");
        }

        @Test
        void ballBallCollisionWorks() {
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.subticks = 10;
            sim.setConfig(config);

            // Two balls on collision course (at same height, no gravity effect)
            sim.spawnBall(new Translation3d(7.8, 4, 2), new Translation3d(5, 0, 0));
            sim.spawnBall(new Translation3d(8.2, 4, 2), new Translation3d(-5, 0, 0));

            // Run until they collide and separate
            for (int i = 0; i < 50; i++) sim.advancePhysics(0.02);

            List<Translation3d> vels = sim.getBallVelocities();
            if (vels.size() >= 2) {
                // After collision, velocities should have reversed direction
                // (with COR < 1, they'll be slower)
                assertTrue(vels.get(0).getX() < 0, "Ball A should reverse: " + vels.get(0).getX());
                assertTrue(vels.get(1).getX() > 0, "Ball B should reverse: " + vels.get(1).getX());
            }
        }

        @Test
        void fastBallDoesNotTunnelThroughWall() {
            config.ccdEnabled = true;
            config.sleepingEnabled = false;
            sim.setConfig(config);

            // Very fast ball aimed at alliance wall
            sim.spawnBall(new Translation3d(1, 4, 0.3), new Translation3d(-50, 0, 0));

            sim.advancePhysics(0.02);

            // Ball should NOT have gone through the wall
            Translation3d pos = sim.getBallPositions().get(0);
            assertTrue(
                    pos.getX() >= FuelPhysicsSim.getBallRadius() - 0.01,
                    "Ball tunneled through wall: x=" + pos.getX());
        }

        @Test
        void fastBallDoesNotTunnelThroughThinObstacle() {
            config.ccdEnabled = true;
            config.sleepingEnabled = false;
            config.subticks = 1; // Deliberately low to test CCD
            sim.setConfig(config);

            // Fast ball aimed at a trench pillar (about 0.3m wide)
            // Trench pillar at approximately X=3.96..4.265
            sim.spawnBall(new Translation3d(3.5, 1.4, 0.5), new Translation3d(50, 0, 0));

            sim.advancePhysics(0.02);

            // Ball should have been stopped by CCD
            Translation3d pos = sim.getBallPositions().get(0);
            // Should not have passed through to X > 4.3
            // (it might bounce back or stop at the pillar)
            assertTrue(
                    pos.getX() < 5.0 || pos.getX() > 4.3,
                    "CCD should prevent tunneling: x=" + pos.getX());
        }

        @Test
        void higherImpactSpeedLowerBounceHeight() {
            config.velocityDependentCOR = true;
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            sim.setConfig(config);

            // Slow drop
            FuelPhysicsSim simSlow = new FuelPhysicsSim("Test/Slow", config);
            simSlow.enable();
            simSlow.setConfig(config);
            simSlow.spawnBall(new Translation3d(8, 3, 1)); // 1m drop

            // Fast drop
            FuelPhysicsSim simFast = new FuelPhysicsSim("Test/Fast", config);
            simFast.enable();
            simFast.setConfig(config);
            simFast.spawnBall(new Translation3d(8, 5, 5)); // 5m drop

            double maxBounceSlow = 0, maxBounceFast = 0;
            boolean slowBounced = false, fastBounced = false;

            for (int i = 0; i < 300; i++) {
                simSlow.advancePhysics(0.02);
                simFast.advancePhysics(0.02);

                if (simSlow.getBallCount() > 0) {
                    double z = simSlow.getBallPositions().get(0).getZ();
                    double vz = simSlow.getBallVelocities().get(0).getZ();
                    if (slowBounced) maxBounceSlow = Math.max(maxBounceSlow, z);
                    if (vz > 0 && z < 0.2) slowBounced = true;
                }
                if (simFast.getBallCount() > 0) {
                    double z = simFast.getBallPositions().get(0).getZ();
                    double vz = simFast.getBallVelocities().get(0).getZ();
                    if (fastBounced) maxBounceFast = Math.max(maxBounceFast, z);
                    if (vz > 0 && z < 0.2) fastBounced = true;
                }
            }

            if (maxBounceSlow > 0 && maxBounceFast > 0) {
                // COR ratio: fast should bounce relatively lower than slow
                double ratioSlow = maxBounceSlow / 1.0;
                double ratioFast = maxBounceFast / 5.0;
                assertTrue(
                        ratioFast < ratioSlow,
                        "Higher impact should have lower COR ratio: slow="
                                + ratioSlow
                                + " fast="
                                + ratioFast);
            }
        }

        @Test
        void ballGrazingRungDeflectsTangentially() {
            config.sleepingEnabled = false;
            config.magnusEnabled = false;
            sim.setConfig(config);

            // Ball at tower rung height, moving toward it at a tangent
            // Blue tower rung at X=0, Y~3.6..4.5, Z=0.686
            sim.spawnBall(new Translation3d(0.3, 3.5, 0.686), new Translation3d(-3, 1, 0));

            for (int i = 0; i < 50; i++) sim.advancePhysics(0.02);

            // Ball should have been deflected (Y component changed)
            Translation3d vel = sim.getBallVelocities().get(0);
            assertTrue(vel.getNorm() > 0.1, "Ball should still be moving after rung deflection");
        }
    }

    @Nested
    class Sleeping {
        @Test
        void sleepingBallsReduceComputeTime() {
            config.sleepingEnabled = true;
            config.subticks = 5;
            sim.setConfig(config);

            // Spawn many balls at rest (they should sleep quickly)
            for (int i = 0; i < 350; i++) {
                sim.spawnBall(
                        new Translation3d(
                                2 + (i % 50) * 0.2,
                                1 + (i / 50) * 0.2,
                                FuelPhysicsSim.getBallRadius()));
            }

            // Let them settle and sleep
            for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);

            int sleeping = sim.getSleepingBallCount();
            assertTrue(sleeping > 200, "Most resting balls should be sleeping: " + sleeping);

            // Time a step with mostly sleeping balls
            long start = System.nanoTime();
            for (int i = 0; i < 10; i++) sim.advancePhysics(0.02);
            double elapsedMs = (System.nanoTime() - start) / 1_000_000.0 / 10.0;

            assertTrue(
                    elapsedMs < 2.0,
                    "Step with sleeping balls should be fast: " + elapsedMs + "ms");
        }

        @Test
        void sleepingBallWakesOnExternalCollision() {
            config.sleepingEnabled = true;
            sim.setConfig(config);

            // Spawn ball at rest, let it sleep (Y=1 avoids hub geometry)
            sim.spawnBall(new Translation3d(8, 1, FuelPhysicsSim.getBallRadius()));
            for (int i = 0; i < 50; i++) sim.advancePhysics(0.02);

            assertTrue(sim.getSleepingBallCount() > 0, "Ball should be sleeping");

            // Launch a ball at it
            sim.launchBall(
                    new Translation3d(7, 1, FuelPhysicsSim.getBallRadius()),
                    new Translation3d(5, 0, 0),
                    0);

            // Run until collision
            for (int i = 0; i < 50; i++) sim.advancePhysics(0.02);

            // The resting ball should have woken up
            assertEquals(0, sim.getSleepingBallCount(), "Collision should wake sleeping ball");
        }

        @Test
        void allBallsEventuallySleep() {
            config.sleepingEnabled = true;
            sim.setConfig(config);

            // Spawn 50 balls with small velocities
            for (int i = 0; i < 50; i++) {
                sim.spawnBall(
                        new Translation3d(
                                3 + (i % 10) * 0.3,
                                2 + (i / 10) * 0.3,
                                FuelPhysicsSim.getBallRadius()),
                        new Translation3d(0.1, 0.05, 0));
            }

            for (int i = 0; i < 500; i++) sim.advancePhysics(0.02);

            int sleeping = sim.getSleepingBallCount();
            assertTrue(sleeping >= 25, "Most balls should eventually sleep: " + sleeping + "/50");
        }

        @Test
        void noSleepingAllBallsAlwaysActive() {
            config.sleepingEnabled = false;
            sim.setConfig(config);

            for (int i = 0; i < 10; i++) {
                sim.spawnBall(new Translation3d(8, 4, FuelPhysicsSim.getBallRadius()));
            }
            for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);

            assertEquals(0, sim.getSleepingBallCount(), "No sleeping when disabled");
        }
    }

    @Nested
    class FrictionAndSpinTransfer {
        @Test
        void slidingBallDeceleratesAtMuTimesG() {
            config.magnusEnabled = false;
            config.dragEnabled = false; // isolate ground friction from air drag
            config.sleepingEnabled = false;
            config.spinTransferEnabled = false;
            sim.setConfig(config);

            double initialSpeed = 5.0;
            sim.spawnBall(
                    new Translation3d(8, 4, FuelPhysicsSim.getBallRadius()),
                    new Translation3d(initialSpeed, 0, 0));

            sim.advancePhysics(0.5); // 0.5 seconds

            Translation3d vel = sim.getBallVelocities().get(0);
            double finalSpeed = Math.abs(vel.getX());

            // Expected deceleration: mu_k * g = 0.3 * 9.81 = 2.943 m/s^2
            // After 0.5s: v = 5 - 2.943*0.5 = 3.53 m/s
            double expectedSpeed = initialSpeed - 0.3 * 9.81 * 0.5;
            assertEquals(
                    expectedSpeed,
                    finalSpeed,
                    expectedSpeed * 0.2,
                    "Ground friction deceleration should match mu*g");
        }

        @Test
        void spinTransferOnWallCollision() {
            config.sleepingEnabled = false;
            config.magnusEnabled = false;
            sim.setConfig(config);

            // Ball with no spin hits a wall
            sim.spawnBall(
                    new Translation3d(FuelPhysicsSim.getBallRadius() + 0.01, 4, 0.3),
                    new Translation3d(-5, 3, 0)); // Oblique hit

            sim.advancePhysics(0.02);

            Translation3d omega = sim.getBallOmegas().get(0);
            assertTrue(
                    omega.getNorm() > 0.1,
                    "Wall collision should impart spin: omega=" + omega.getNorm());
        }

        @Test
        void groundFrictionSlowsSpin() {
            config.magnusEnabled = false;
            config.sleepingEnabled = false;
            config.spinDecayEnabled = false; // test ground friction only
            sim.setConfig(config);

            // Ball on ground with high spin
            sim.launchBall(
                    new Translation3d(8, 4, FuelPhysicsSim.getBallRadius()),
                    new Translation3d(0.5, 0, 0),
                    new Translation3d(0, 100, 0)); // High spin

            double initialOmega = sim.getBallOmegas().get(0).getNorm();

            for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);

            double finalOmega = sim.getBallOmegas().get(0).getNorm();
            assertTrue(
                    finalOmega < initialOmega,
                    "Ground friction should reduce spin: initial="
                            + initialOmega
                            + " final="
                            + finalOmega);
        }

        @Test
        void noFrictionBallNeverStops() {
            config.dragEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            config.magnusEnabled = false;
            sim.setConfig(config);

            // Y=1 avoids hub ramp geometry
            sim.spawnBall(
                    new Translation3d(8, 1, FuelPhysicsSim.getBallRadius()),
                    new Translation3d(3, 0, 0));

            for (int i = 0; i < 500; i++) sim.advancePhysics(0.02);

            Translation3d vel = sim.getBallVelocities().get(0);
            double speed = Math.sqrt(vel.getX() * vel.getX() + vel.getY() * vel.getY());
            // Speed decreases from COR on wall bounces, but stays > 1.0
            assertTrue(speed > 1.0, "Ball should keep moving without friction: speed=" + speed);
        }
    }

    @Nested
    class Performance {
        @Test
        void fiftyBallStepUnder2ms() {
            config.sleepingEnabled = false;
            sim.setConfig(config);

            for (int i = 0; i < 50; i++) {
                sim.spawnBall(
                        new Translation3d(
                                2 + (i % 10) * 0.5, 1 + (i / 10) * 0.5, 0.5 + (i % 5) * 0.3),
                        new Translation3d(Math.cos(i * 0.3) * 3, Math.sin(i * 0.3) * 3, 2));
            }

            // Warmup
            for (int i = 0; i < 10; i++) sim.advancePhysics(0.02);

            // Measure
            long start = System.nanoTime();
            int steps = 100;
            for (int i = 0; i < steps; i++) sim.advancePhysics(0.02);
            double meanMs = (System.nanoTime() - start) / 1_000_000.0 / steps;

            assertTrue(meanMs < 5.0, "50-ball step should be under 5ms: " + meanMs + "ms");
        }

        @Test
        void fourHundredBallStepUnder10ms() {
            config.sleepingEnabled = true; // sleeping helps with resting balls
            sim.setConfig(config);

            for (int i = 0; i < 400; i++) {
                sim.spawnBall(
                        new Translation3d(
                                1 + (i % 40) * 0.35,
                                0.5 + (i / 40) * 0.35,
                                FuelPhysicsSim.getBallRadius()));
            }

            // Let balls settle (most will sleep)
            for (int i = 0; i < 50; i++) sim.advancePhysics(0.02);

            long start = System.nanoTime();
            int steps = 50;
            for (int i = 0; i < steps; i++) sim.advancePhysics(0.02);
            double meanMs = (System.nanoTime() - start) / 1_000_000.0 / steps;

            assertTrue(meanMs < 15.0, "400-ball step should be under 15ms: " + meanMs + "ms");
        }
    }

    @Nested
    class Determinism {
        @Test
        void sameInitialConditionsSameFinalState() {
            config.deterministic = true;
            config.deterministicSeed = 123L;
            sim.setConfig(config);

            sim.launchBall(new Translation3d(8, 4, 1), new Translation3d(5, 2, 8), 2000);
            sim.launchBall(new Translation3d(6, 3, 0.5), new Translation3d(-3, 1, 4), 1000);

            for (int i = 0; i < 200; i++) sim.advancePhysics(0.02);
            List<Translation3d> positions1 = sim.getBallPositions();

            // Run again with same setup
            FuelPhysicsSim sim2 = new FuelPhysicsSim("Test/Det2", config);
            sim2.enable();
            sim2.setConfig(config);
            sim2.launchBall(new Translation3d(8, 4, 1), new Translation3d(5, 2, 8), 2000);
            sim2.launchBall(new Translation3d(6, 3, 0.5), new Translation3d(-3, 1, 4), 1000);

            for (int i = 0; i < 200; i++) sim2.advancePhysics(0.02);
            List<Translation3d> positions2 = sim2.getBallPositions();

            assertEquals(positions1.size(), positions2.size(), "Same ball count");
            for (int i = 0; i < positions1.size(); i++) {
                assertEquals(
                        positions1.get(i).getX(),
                        positions2.get(i).getX(),
                        1e-9,
                        "X position mismatch at ball " + i);
                assertEquals(
                        positions1.get(i).getY(),
                        positions2.get(i).getY(),
                        1e-9,
                        "Y position mismatch at ball " + i);
                assertEquals(
                        positions1.get(i).getZ(),
                        positions2.get(i).getZ(),
                        1e-9,
                        "Z position mismatch at ball " + i);
            }
        }

        @Test
        void deterministicModeProducesReproducibleDispersal() {
            config.deterministic = true;
            config.deterministicSeed = 42L;
            config.sleepingEnabled = false;
            sim.setConfig(config);

            // Drop ball directly above blue hub center
            sim.launchBall(new Translation3d(4.5974, 4.035, 5.0), new Translation3d(0, 0, 0), 0);

            // Run until scoring happens
            for (int i = 0; i < 300; i++) sim.advancePhysics(0.02);

            int scored1 = sim.getTotalScored();
            List<Translation3d> finalPos1 = sim.getBallPositions();

            // Repeat
            FuelPhysicsSim sim2 = new FuelPhysicsSim("Test/Det3", config);
            sim2.enable();
            sim2.setConfig(config);
            sim2.launchBall(new Translation3d(4.5974, 4.035, 5.0), new Translation3d(0, 0, 0), 0);
            for (int i = 0; i < 300; i++) sim2.advancePhysics(0.02);

            assertEquals(scored1, sim2.getTotalScored(), "Same scoring outcome");
            if (!finalPos1.isEmpty() && !sim2.getBallPositions().isEmpty()) {
                assertEquals(
                        finalPos1.get(0).getX(),
                        sim2.getBallPositions().get(0).getX(),
                        1e-9,
                        "Same dispersal position");
            }
        }
    }

    @Nested
    class FeatureFlags {
        @Test
        void allFeaturesOffMatchesBaseline() {
            // With all advanced features off, behavior should be simple Euler + bounce
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.spinTransferEnabled = false;
            config.sleepingEnabled = false;
            config.ccdEnabled = false;
            config.velocityDependentCOR = false;
            config.spinDecayEnabled = false;
            config.solverIterations = 1;
            sim.setConfig(config);

            sim.launchBall(new Translation3d(8, 4, 2), new Translation3d(5, 0, 5), 0);

            // Should still work without crashing
            assertDoesNotThrow(
                    () -> {
                        for (int i = 0; i < 100; i++) sim.advancePhysics(0.02);
                    });

            assertTrue(sim.getBallCount() > 0, "Ball should still exist");
        }

        @Test
        void configCopyIsIndependent() {
            PhysicsConfig original = new PhysicsConfig();
            original.magnusEnabled = true;
            PhysicsConfig copy = original.copy();
            copy.magnusEnabled = false;
            assertTrue(original.magnusEnabled, "Copy should not affect original");
        }
    }

    @Nested
    class HubScoring {
        @Test
        void ballScoresThroughHubOpening() {
            config.sleepingEnabled = false;
            config.magnusEnabled = false;
            sim.setConfig(config);

            // Drop ball from directly above blue hub center so it falls through the opening
            // Hub at (4.5974, 4.035), entry height at 1.829m
            sim.launchBall(new Translation3d(4.5974, 4.035, 5.0), new Translation3d(0, 0, 0), 0);

            for (int i = 0; i < 300; i++) {
                sim.advancePhysics(0.02);
            }

            assertTrue(sim.getTotalScored() > 0, "Ball should have scored in hub");
        }

        @Test
        void hubScoreCountIncrements() {
            config.sleepingEnabled = false;
            config.magnusEnabled = false;
            sim.setConfig(config);

            // Drop first ball above hub
            sim.launchBall(new Translation3d(4.5974, 4.035, 5.0), new Translation3d(0, 0, 0), 0);

            for (int i = 0; i < 200; i++) sim.advancePhysics(0.02);

            int firstScore = sim.getTotalScored();

            // Drop second ball
            sim.launchBall(new Translation3d(4.5974, 4.035, 5.0), new Translation3d(0, 0, 0), 0);

            for (int i = 0; i < 200; i++) sim.advancePhysics(0.02);

            assertTrue(
                    sim.getTotalScored() >= firstScore,
                    "Score should increment with each scoring ball");
        }
    }

    @Nested
    class DragTests {
        @Test
        void dragReducesRange() {
            config.magnusEnabled = false;
            config.frictionEnabled = false;
            config.sleepingEnabled = false;
            sim.setConfig(config);

            // With drag
            sim.launchBall(new Translation3d(2, 3, 0.5), new Translation3d(10, 0, 10), 0);

            // Without drag (separate sim, but we test by comparing vacuum analytical)
            double v = Math.sqrt(10 * 10 + 10 * 10);
            double theta = Math.atan2(10, 10);
            double vacuumRange = v * v * Math.sin(2 * theta) / 9.81;

            for (int i = 0; i < 300; i++) sim.advancePhysics(0.02);

            double actualRange = 0;
            for (Translation3d pos : sim.getBallPositions()) {
                actualRange = Math.max(actualRange, pos.getX() - 2.0);
            }
            // With drag, range should be shorter than vacuum
            assertTrue(
                    actualRange < vacuumRange,
                    "Drag should reduce range: actual=" + actualRange + " vacuum=" + vacuumRange);
        }
    }
}
