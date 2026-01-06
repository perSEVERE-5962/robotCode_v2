package frc.robot.subsystems;

import frc.robot.Constants.MBLinearSlideConstants;

/* Mini Build linear slide */
public class MBLinearSlide extends Actuator {
    private static MBLinearSlide instance;

    private MBLinearSlide() {
        super(MBLinearSlideConstants.kWristID, MBLinearSlideConstants.kP, MBLinearSlideConstants.kI,
                MBLinearSlideConstants.kD, MBLinearSlideConstants.kMinOutput, MBLinearSlideConstants.kMaxOutput,
                MBLinearSlideConstants.kFF, MBLinearSlideConstants.kIz, MBLinearSlideConstants.kUpperSoftLimit,
                MBLinearSlideConstants.kLowerSoftLimit, true, true, true);
    }

    public static MBLinearSlide getInstance() {
        if (instance == null) {
            instance = new MBLinearSlide();
        }
        return instance;
    }

}
