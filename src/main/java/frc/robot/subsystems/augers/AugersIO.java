package frc.robot.subsystems.augers;

import org.littletonrobotics.junction.AutoLog;

public interface AugersIO {
  @AutoLog
  public static class AugersIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(AugersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
