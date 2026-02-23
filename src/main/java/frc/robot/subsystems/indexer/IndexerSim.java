package frc.robot.subsystems.indexer;

public class IndexerSim implements IndexerIO {
  private double appliedVolts = 0.0;
  private double augersVelocity = 0.0;
  private double kickerVelocity = 0.0;

  // FIXME: Totally made up constant
  private static final double kV = 600.0;

  public IndexerSim() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // FIXME: This sets velocity instantaneously. If we need to simulate
    // ramp-up time, then we need more details here.
    augersVelocity = appliedVolts * kV;

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPM = augersVelocity;
    inputs.kickerVelocity = kickerVelocity;
  }

  @Override
  public void setVelocityAugers(double RPM) {
    augersVelocity = RPM;
  }

  @Override
  public void setVelocityKicker(double RPM) {
    kickerVelocity = RPM;
  }

  @Override
  public void stop() {
    appliedVolts = 0;
  }
}
