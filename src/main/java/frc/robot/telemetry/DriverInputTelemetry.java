package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/** Driver/operator controller input logging and idle tracking. */
public class DriverInputTelemetry implements SubsystemTelemetry {
    private XboxController driver;
    private XboxController operator;

    // Driver input state
    private double driverLeftX, driverLeftY, driverRightX, driverRightY;
    private double driverLeftTrigger, driverRightTrigger;
    private int driverButtons;
    private int driverPOV;

    // Operator input state
    private double operatorLeftX, operatorLeftY, operatorRightX, operatorRightY;
    private double operatorLeftTrigger, operatorRightTrigger;
    private int operatorButtons;
    private int operatorPOV;

    // Analytics
    private double driverInputMagnitude = 0;
    private double operatorInputMagnitude = 0;
    private boolean driverActive = false;
    private boolean operatorActive = false;
    private double driverIdleTimeMs = 0;
    private double lastDriverInputTime = -1;  // M2: Sentinel value to detect first update

    private boolean deadbandActiveX = false;
    private boolean deadbandActiveY = false;
    private double operatorIdleTimeMs = 0;
    private double lastOperatorInputTime = -1;

    // Thresholds
    private static final double DEADBAND = 0.1;

    public DriverInputTelemetry() {
        // Controllers will be set via setControllers()
    }

    /** Called from RobotContainer. */
    public void setControllers(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void update() {
        double now = Timer.getFPGATimestamp();

        if (driver == null) {
            return;
        }

        // Driver inputs
        driverLeftX = driver.getLeftX();
        driverLeftY = driver.getLeftY();
        driverRightX = driver.getRightX();
        driverRightY = driver.getRightY();
        driverLeftTrigger = driver.getLeftTriggerAxis();
        driverRightTrigger = driver.getRightTriggerAxis();
        driverPOV = driver.getPOV();

        // Pack driver buttons as bitmask
        driverButtons = packButtons(
            driver.getAButton(), driver.getBButton(),
            driver.getXButton(), driver.getYButton(),
            driver.getLeftBumper(), driver.getRightBumper(),
            driver.getStartButton(), driver.getBackButton()
        );

        // Operator inputs (if available)
        if (operator != null) {
            operatorLeftX = operator.getLeftX();
            operatorLeftY = operator.getLeftY();
            operatorRightX = operator.getRightX();
            operatorRightY = operator.getRightY();
            operatorLeftTrigger = operator.getLeftTriggerAxis();
            operatorRightTrigger = operator.getRightTriggerAxis();
            operatorPOV = operator.getPOV();

            operatorButtons = packButtons(
                operator.getAButton(), operator.getBButton(),
                operator.getXButton(), operator.getYButton(),
                operator.getLeftBumper(), operator.getRightBumper(),
                operator.getStartButton(), operator.getBackButton()
            );
        }

        driverInputMagnitude = Math.sqrt(
            driverLeftX * driverLeftX + driverLeftY * driverLeftY +
            driverRightX * driverRightX + driverRightY * driverRightY
        );

        operatorInputMagnitude = Math.sqrt(
            operatorLeftX * operatorLeftX + operatorLeftY * operatorLeftY
        );

        driverActive = driverInputMagnitude > DEADBAND ||
                       driverLeftTrigger > DEADBAND ||
                       driverRightTrigger > DEADBAND ||
                       driverButtons != 0;

        operatorActive = operatorInputMagnitude > DEADBAND ||
                        operatorButtons != 0;

        // Track driver idle time
        // M2: Initialize on first update to avoid FPGA uptime spike
        if (lastDriverInputTime < 0) {
            lastDriverInputTime = now;
        }

        if (driverActive) {
            lastDriverInputTime = now;
            driverIdleTimeMs = 0;
        } else {
            driverIdleTimeMs = (now - lastDriverInputTime) * 1000;
        }

        // Deadband detection: input is nonzero but within deadband (driver fighting deadband)
        deadbandActiveX = Math.abs(driverLeftX) < DEADBAND && Math.abs(driverLeftX) > 0.01;
        deadbandActiveY = Math.abs(driverLeftY) < DEADBAND && Math.abs(driverLeftY) > 0.01;

        // Operator idle time tracking
        if (lastOperatorInputTime < 0) lastOperatorInputTime = now;
        if (operatorActive) {
            lastOperatorInputTime = now;
            operatorIdleTimeMs = 0;
        } else {
            operatorIdleTimeMs = (now - lastOperatorInputTime) * 1000;
        }
    }

    @Override
    public void log() {
        if (driver == null) {
            return;
        }

        // Driver joysticks
        SafeLog.put("Driver/LeftStick/X", driverLeftX);
        SafeLog.put("Driver/LeftStick/Y", driverLeftY);
        SafeLog.put("Driver/RightStick/X", driverRightX);
        SafeLog.put("Driver/RightStick/Y", driverRightY);
        SafeLog.put("Driver/LeftTrigger", driverLeftTrigger);
        SafeLog.put("Driver/RightTrigger", driverRightTrigger);
        SafeLog.put("Driver/Buttons", driverButtons);
        SafeLog.put("Driver/POV", driverPOV);

        // Driver analytics
        SafeLog.put("Driver/InputMagnitude", driverInputMagnitude);
        SafeLog.put("Driver/Active", driverActive);
        SafeLog.put("Driver/IdleTimeMs", driverIdleTimeMs);

        // Operator joysticks
        SafeLog.put("Operator/LeftStick/X", operatorLeftX);
        SafeLog.put("Operator/LeftStick/Y", operatorLeftY);
        SafeLog.put("Operator/RightStick/X", operatorRightX);
        SafeLog.put("Operator/RightStick/Y", operatorRightY);
        SafeLog.put("Operator/LeftTrigger", operatorLeftTrigger);
        SafeLog.put("Operator/RightTrigger", operatorRightTrigger);
        SafeLog.put("Operator/Buttons", operatorButtons);
        SafeLog.put("Operator/POV", operatorPOV);

        // Operator analytics
        SafeLog.put("Operator/InputMagnitude", operatorInputMagnitude);
        SafeLog.put("Operator/Active", operatorActive);

        SafeLog.put("Driver/DeadbandActiveX", deadbandActiveX);
        SafeLog.put("Driver/DeadbandActiveY", deadbandActiveY);
        SafeLog.put("Operator/IdleTimeMs", operatorIdleTimeMs);
    }

    @Override
    public String getName() {
        return "DriverInput";
    }

    // Accessors
    public boolean isDriverActive() { return driverActive; }
    public boolean isOperatorActive() { return operatorActive; }
    public double getDriverIdleTimeMs() { return driverIdleTimeMs; }

    private int packButtons(boolean a, boolean b, boolean x, boolean y,
                           boolean lb, boolean rb, boolean start, boolean back) {
        return (a ? 1 : 0) | (b ? 2 : 0) | (x ? 4 : 0) | (y ? 8 : 0) |
               (lb ? 16 : 0) | (rb ? 32 : 0) | (start ? 64 : 0) | (back ? 128 : 0);
    }
}
