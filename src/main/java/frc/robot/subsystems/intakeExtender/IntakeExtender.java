package frc.robot.subsystems.intakeExtender;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeExtender extends SubsystemBase {
  private final IntakeExtenderIO intakeIO;
  private final IntakeExtenderIOInputsAutoLogged intakeExtenderInputs =
      new IntakeExtenderIOInputsAutoLogged();

  private double extendAngle = 0.0;
  private double outputCurrent = 0.0;

  public IntakeExtender(IntakeExtenderIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeExtenderInputs);
    Logger.processInputs("Intake", intakeExtenderInputs);
    extendAngle = intakeExtenderInputs.extendPosition;
    outputCurrent = intakeExtenderInputs.outputCurrent;
  }

  public double getExtendAngle() {
    return extendAngle;
  }

  public void stop() {
    intakeIO.extend(0.0);
  }

  public boolean motorCurrentWithinLimit() {
    return outputCurrent < IntakeExtenderConstants.CURRENT_LIMIT;
  }

  public void extend(double dutyCycle) {
    intakeIO.extend(dutyCycle);
  }
}
