package frc.robot.subsystems.indexer;

import frc.robot.util.SimUtils;

public class IndexerSim implements IndexerIO {
  private double augersVelocity = 0.0;
  private double kickerVelocity = 0.0;
  private double augersSetpoint = 0.0;
  private double kickerSetpoint = 0.0;
  private boolean stopped = false;

  private static final double AUGERS_ACCEL_RATE = 300.0; // RPM per second
  private static final double KICKER_ACCEL_RATE = 1000.0; // RPM per second

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if (!stopped) {
      augersVelocity = SimUtils.slew(augersVelocity, augersSetpoint, AUGERS_ACCEL_RATE);
      kickerVelocity = SimUtils.slew(kickerVelocity, kickerSetpoint, KICKER_ACCEL_RATE);
    } else {
      augersVelocity = SimUtils.slew(augersVelocity, 0.0, AUGERS_ACCEL_RATE);
      kickerVelocity = SimUtils.slew(kickerVelocity, 0.0, KICKER_ACCEL_RATE);
    }

    inputs.augerVelocityRPM = augersVelocity;
    inputs.augerAppliedVolts = (augersVelocity / 6000.0) * 12.0;
    inputs.kickerVelocity = kickerVelocity;
    inputs.indexerStatus = true;
  }

  @Override
  public void setVelocityAugers(double RPM) {
    stopped = false;
    augersSetpoint = RPM;
  }

  @Override
  public void setVelocityKicker(double RPM) {
    stopped = false;
    kickerSetpoint = RPM;
  }

  @Override
  public void stop() {
    stopped = true;
  }
}
