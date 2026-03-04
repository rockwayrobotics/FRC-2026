package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerAppliedVolts = 0.0;
    public double rollerVelocity = 0.0;
    public double rollerCurrent = 0.0;
    public boolean intakeStatus = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void intake(double dutyCycle) {}
}
