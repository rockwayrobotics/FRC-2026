package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double position = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setPose(double pose) {}
}
