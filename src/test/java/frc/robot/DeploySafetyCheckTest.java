package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Validates deploy safety gates catch the current state of TUNING_MODE. */
class DeploySafetyCheckTest {

  @Test
  void isSafeForDeployMatchesTuningMode() {
    // isSafeForDeploy should return false when TUNING_MODE is true, and vice versa
    assertEquals(!Constants.TUNING_MODE, Constants.DeploySafetyCheck.isSafeForDeploy());
  }

  @Test
  void tuningModeCurrentlyTrue() {
    // This test documents that TUNING_MODE is currently true (development mode).
    // When set to false for competition, flip this assertion and the one above.
    assertTrue(Constants.TUNING_MODE, "TUNING_MODE should be true during development");
  }
}
