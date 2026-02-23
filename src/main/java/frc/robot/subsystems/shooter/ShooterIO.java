package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelVelocity = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double hoodPosition = 0.0;
    public boolean shooterStatus = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVelocityFlywheel(double RPM) {}

  public default void setPositionHood(Angle angle) {}

  public default void stopHood() {}

  public default void stopFlywheel() {}
}
