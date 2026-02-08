package frc.robot.util;

import frc.robot.Constants.OperatorConstants;

/** Tunable driver input parameters for practice. */
public final class DriverTuning {
    private DriverTuning() {} // Utility class

    // Driver input tunables
    public static final TunableNumber deadband =
        new TunableNumber("Driver/Deadband", OperatorConstants.DEADBAND);

    public static final TunableNumber turnConstant =
        new TunableNumber("Driver/TurnConstant", OperatorConstants.TURN_CONSTANT);

    public static void initialize() {
        double d = deadband.get();
        double t = turnConstant.get();
    }
}
