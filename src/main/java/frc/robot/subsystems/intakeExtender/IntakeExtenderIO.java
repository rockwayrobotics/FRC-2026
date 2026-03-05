package frc.robot.subsystems.intakeExtender;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtenderIO {
  @AutoLog
  public static class IntakeExtenderIOInputs {
    public double extendPosition = 0.0;
    public double appliedVolts = 0.0;
    public double outputCurrent = 0.0;
    public boolean intakeExtenderStatus = false;
  }

  public default void updateInputs(IntakeExtenderIOInputs inputs) {}

  public default void extend(double dutyCycle) {}
}
