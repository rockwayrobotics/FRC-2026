package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelVelocity = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double hoodPosition = 0.0;
    public double kickerVelocity = 0.0;
    public boolean shooterStatus = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltageShooter(double voltage) {}

  public default void setVoltageKicker(double voltage) {}

  public default void setVoltageHood(double voltage) {}
}
