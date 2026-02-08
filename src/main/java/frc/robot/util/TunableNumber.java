package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** Dashboard-tunable number. Returns compile-time default when TUNING_MODE is off. */
public class TunableNumber implements DoubleSupplier {
    private final String key;
    private final double defaultValue;
    private double lastValue;
    private boolean initialized = false;

    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;

        if (Constants.TUNING_MODE) {
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    /** Current value (from dashboard if tuning, else default). */
    public double get() {
        if (!Constants.TUNING_MODE) {
            return defaultValue;
        }

        if (!initialized) {
            initialized = true;
            lastValue = SmartDashboard.getNumber(key, defaultValue);
        }

        return SmartDashboard.getNumber(key, defaultValue);
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    /** True if value changed since last check. */
    public boolean hasChanged() {
        if (!Constants.TUNING_MODE) {
            return false;
        }

        double currentValue = get();
        if (currentValue != lastValue) {
            lastValue = currentValue;
            return true;
        }
        return false;
    }

    public String getKey() {
        return key;
    }

    public double getDefault() {
        return defaultValue;
    }

    public void reset() {
        if (Constants.TUNING_MODE) {
            SmartDashboard.putNumber(key, defaultValue);
        }
        lastValue = defaultValue;
    }

    public static boolean tuningEnabled() {
        return Constants.TUNING_MODE;
    }

    /** Run action if any tunable changed. */
    public static void ifChanged(Runnable action, TunableNumber... tunables) {
        boolean anyChanged = false;
        for (TunableNumber t : tunables) {
            if (t.hasChanged()) {
                anyChanged = true;
            }
        }
        if (anyChanged) {
            action.run();
        }
    }
}
