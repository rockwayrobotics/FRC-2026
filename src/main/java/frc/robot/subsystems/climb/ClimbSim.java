package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;

public class ClimbSim implements ClimbIO {
  private double appliedVolts = 0.0;
  private double velocityRPM = 0.0;

  public ClimbSim() {}

  @Override
  public void updateInputs(ClimbIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
