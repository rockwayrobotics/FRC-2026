package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double retractPosition = 0.0;
    public double retractAppliedVolts = 0.0;
    public double fixedAppliedVolts = 0.0;
    public double fixedVelocityRadsPerSec = 0.0;
    public double extendingAppliedVolts = 0.0;
    public double extendingVelocityRadsPerSec = 0.0;
    public double rollerVelocity = 0.0;
    public double rollerCurrent = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
