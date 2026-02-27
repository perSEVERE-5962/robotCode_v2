package frc.robot.subsystems;

import frc.robot.Constants;

public class Indexer extends Actuator {
  private static Indexer instance;

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
