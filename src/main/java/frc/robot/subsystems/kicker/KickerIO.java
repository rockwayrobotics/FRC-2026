package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double kickerVelocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean kickerStatus = false;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void stop() {}

  public default void setVelocityKicker(double RPM) {}
}
