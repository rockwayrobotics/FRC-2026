package frc.robot.subsystems.augers;

import edu.wpi.first.math.MathUtil;

public class AugersSim implements AugersIO {
  private double appliedVolts = 0.0;
  private double velocityRPM = 0.0;

  // FIXME: Totally made up constant
  private static final double kV = 600.0;

  public AugersSim() {}

  @Override
  public void updateInputs(AugersIOInputs inputs) {
    // FIXME: This sets velocity instantaneously. If we need to simulate
    // ramp-up time, then we need more details here.
    velocityRPM = appliedVolts * kV;

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPM = velocityRPM;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
