package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.TunableNumber;

public class Indexer extends Actuator {
  private static Indexer instance;

  // Tunable PID values
  private static final TunableNumber kP = new TunableNumber("Indexer/kP", IndexerConstants.P);
  private static final TunableNumber kI = new TunableNumber("Indexer/kI", IndexerConstants.I);
  private static final TunableNumber kD = new TunableNumber("Indexer/kD", IndexerConstants.D);
  private static final TunableNumber kF = new TunableNumber("Indexer/FF", IndexerConstants.FF);

  // Tunable operational values
  private static final TunableNumber targetSpeed =
      new TunableNumber("Indexer/TargetSpeed", IndexerConstants.TARGET_SPEED);
  private static final TunableNumber jamCurrentThreshold =
      new TunableNumber("Indexer/JamAmps", IndexerConstants.JAM_CURRENT_THRESHOLD_AMPS);
  private static final TunableNumber jamTimeThreshold =
      new TunableNumber("Indexer/JamSeconds", IndexerConstants.JAM_TIME_THRESHOLD_SECONDS);

  private Indexer() {
    super(Constants.CANDeviceIDs.kIndexerID, Constants.IndexerConstants.P, Constants.IndexerConstants.I, Constants.IndexerConstants.D, Constants.IndexerConstants.MinOutput, Constants.IndexerConstants.MaxOutput, Constants.IndexerConstants.FF, Constants.IndexerConstants.Iz, 0, 0, 40, true, true, false, false);
  }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }
}
