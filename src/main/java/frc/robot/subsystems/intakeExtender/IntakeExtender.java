package frc.robot.subsystems.intakeExtender;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeExtender extends SubsystemBase {
  private final IntakeExtenderIO intakeIO;
  private final IntakeExtenderIOInputsAutoLogged intakeExtenderInputs =
      new IntakeExtenderIOInputsAutoLogged();

  private double extendAngle = 0.0;
  private double outputCurrent = 0.0;
  private boolean retractBlocked = false;
  private boolean holdingPosition = false;

  public IntakeExtender(IntakeExtenderIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeExtenderInputs);
    Logger.processInputs("Intake", intakeExtenderInputs);
    extendAngle = intakeExtenderInputs.extendPosition;
    outputCurrent = intakeExtenderInputs.outputCurrent;
    if (intakeExtenderInputs.appliedVolts < 0) {
      // Attempting to retract
      if (!motorCurrentWithinLimit()) {
        retractBlocked = true;
      }
    } else if (intakeExtenderInputs.appliedVolts > 0) {
      // Attempting to extend
      retractBlocked = false;
    }
  }

  public double getExtendAngle() {
    return extendAngle;
  }

  public boolean isAutoRetractBlocked() {
    return retractBlocked;
  }

  public boolean isHoldingPosition() {
    return holdingPosition;
  }

  public void setHoldingPosition(boolean holdingPosition) {
    this.holdingPosition = holdingPosition;
  }

  public void unlockAutoRetract() {
    retractBlocked = false;
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
