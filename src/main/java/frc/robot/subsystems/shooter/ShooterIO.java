package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelVelocity = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double hoodPosition = 0.0;
    public double kickerVelocity = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
