package frc.robot.subsystems.indexer;

import frc.robot.util.SimUtils;

public class IndexerSim implements IndexerIO {
  private double augersVelocity = 0.0;
  private double augersSetpoint = 0.0;
  private boolean stopped = false;

  private static final double AUGERS_ACCEL_RATE = 300.0; // RPM per second

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if (!stopped) {
      augersVelocity = SimUtils.slew(augersVelocity, augersSetpoint, AUGERS_ACCEL_RATE);
    } else {
      augersVelocity = SimUtils.slew(augersVelocity, 0.0, AUGERS_ACCEL_RATE);
    }

    inputs.augerVelocityRPM = augersVelocity;
    inputs.augerAppliedVolts = (augersVelocity / 6000.0) * 12.0;
    inputs.indexerStatus = true;
  }

  @Override
  public void setAugers(double value) {
    augersSetpoint = 2000 * value;
    augersVelocity = augersSetpoint;
  }

  @Override
  public void setVelocityAugers(double RPM) {
    stopped = false;
    augersSetpoint = RPM;
  }

  @Override
  public void stop() {
    stopped = true;
  }
}
