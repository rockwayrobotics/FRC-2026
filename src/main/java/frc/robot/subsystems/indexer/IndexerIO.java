package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double augerVelocityRPM = 0.0;
    public double augerAppliedVolts = 0.0;
    public double kickerVelocity = 0.0;
    public boolean indexerStatus = false;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void stop() {}

  public default void setVelocityAugers(double RPM) {}

  public default void setVelocityKicker(double RPM) {}
}
