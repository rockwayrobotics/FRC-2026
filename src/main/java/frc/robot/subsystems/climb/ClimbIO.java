package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public boolean limitsEnabled = true;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void stop() {}

  public default void setPos(double pose, boolean fast) {}

  public default void dutyCycle(double value) {}

  public default void configure(boolean limitsEnabled) {}

  public default boolean getLimitsEnabled() {
    return true;
  }
}
