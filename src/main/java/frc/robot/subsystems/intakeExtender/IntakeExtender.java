package frc.robot.subsystems.intakeExtender;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeExtender extends SubsystemBase {
  private final IntakeExtenderIO intakeIO;
  private final IntakeExtenderIOInputsAutoLogged intakeExtenderInputs =
      new IntakeExtenderIOInputsAutoLogged();

  private double extendAngle = 0.0;
  private double appliedVolts = 0.0;

  public IntakeExtender(IntakeExtenderIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeExtenderInputs);
    Logger.processInputs("Intake", intakeExtenderInputs);
    extendAngle = intakeExtenderInputs.extendPosition;
    appliedVolts = intakeExtenderInputs.appliedVolts;
  }

  public double getExtendAngle() {
    return extendAngle;
  }

  public boolean motorCurrentWithinLimit() {
    return appliedVolts < IntakeExtenderConstants.CURRENT_LIMIT;
  }

  public void extend(double dutyCycle) {
    intakeIO.extend(dutyCycle);
  }
}
