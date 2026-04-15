package frc.robot.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Regression tests for the encoder disagreement math in DriveTelemetry. Each test guards one of the
 * three failure modes the WPI Day 1 log surfaced: degrees mistaken for radians, NaN propagating
 * through as a garbage value, and a wraparound case that must collapse to the short arc.
 */
class DriveTelemetryEncoderHealthTest {

  private static final double TOLERANCE_RAD = 0.001;

  @Test
  void fiveDegreeDisagreementReadsAsFiveDegrees() {
    double result = DriveTelemetry.computeDisagreementRad(45.0, 40.0);
    assertEquals(Math.toRadians(5.0), result, TOLERANCE_RAD);
  }

  @Test
  void nanAbsoluteReturnsZeroNotGarbage() {
    double result = DriveTelemetry.computeDisagreementRad(Double.NaN, 90.0);
    assertEquals(0.0, result, 0.0);
    assertFalse(Double.isNaN(result));
    assertFalse(Double.isInfinite(result));
  }

  @Test
  void nanRelativeReturnsZeroNotGarbage() {
    double result = DriveTelemetry.computeDisagreementRad(90.0, Double.NaN);
    assertEquals(0.0, result, 0.0);
    assertFalse(Double.isNaN(result));
  }

  @Test
  void threeHundredSixtyDegreeWraparoundCollapsesToShortArc() {
    double result = DriveTelemetry.computeDisagreementRad(359.0, 1.0);
    assertEquals(Math.toRadians(2.0), result, TOLERANCE_RAD);
  }

  @Test
  void oppositeHemispheresReadsAsPi() {
    double result = DriveTelemetry.computeDisagreementRad(0.0, 180.0);
    assertEquals(Math.PI, result, TOLERANCE_RAD);
  }

  @Test
  void zeroDisagreementReadsAsZero() {
    double result = DriveTelemetry.computeDisagreementRad(123.456, 123.456);
    assertEquals(0.0, result, TOLERANCE_RAD);
  }

  @Test
  void steadyStateNoiseStaysBelowOneTenthRadian() {
    // Typical steady-state: a real robot sits within 1-2 degrees of agreement.
    // None of these should approach the 0.175 rad (10 deg) alert threshold.
    double[] realWorldSamples = {0.5, 1.2, 0.8, 2.1, 1.5, 0.3};
    for (double deg : realWorldSamples) {
      double result = DriveTelemetry.computeDisagreementRad(90.0 + deg, 90.0);
      assertTrue(result < 0.1, "steady state sample " + deg + " deg should stay below 0.1 rad");
    }
  }
}
