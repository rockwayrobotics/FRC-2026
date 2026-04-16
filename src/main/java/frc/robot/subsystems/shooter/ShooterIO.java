package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelVelocity = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public boolean shooterStatus = false;
    public double flywheelLeaderCurrent = 0.0;
    public double flywheelFollower2Current = 0.0;
    public double flywheelLeaderAppliedOutput = 0.0;
    public double flywheelFollower2AppliedOutput = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVelocityFlywheel(double RPM) {}

  public default void stopFlywheel() {}

  public default void configureLeader(double kp, double ki, double kd, double kv) {}
}
