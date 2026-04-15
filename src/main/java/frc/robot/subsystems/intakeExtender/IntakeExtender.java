package frc.robot.subsystems.intakeExtender;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeExtender extends SubsystemBase {
  private final IntakeExtenderIO intakeIO;
  private final IntakeExtenderIOInputsAutoLogged intakeExtenderInputs =
      new IntakeExtenderIOInputsAutoLogged();

  private double extendAngle = 0.0;
  private double outputCurrent = 0.0;

  private boolean autoControlled = false;
  private boolean autoExtending = false;

  public IntakeExtender(IntakeExtenderIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    if (DriverStation.isAutonomous()) {
      if (autoControlled) {
        if (autoExtending) {
          // extend until setpoint and then stay
          if (this.getExtendAngle() < IntakeExtenderConstants.EXTEND_LIMIT) {
            this.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
          } else {
            this.extend(0.0);
          }
        } else {
          // retract until setpoint and then stay
          if (this.getExtendAngle() > IntakeExtenderConstants.RETRACT_LIMIT) {
            this.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
          } else {
            this.extend(0.0);
          }
        }
      }
    }

    intakeIO.updateInputs(intakeExtenderInputs);
    Logger.processInputs("Intake", intakeExtenderInputs);
    extendAngle = intakeExtenderInputs.extendPosition;
    outputCurrent = intakeExtenderInputs.outputCurrent;
  }

  public void setAutoControlled(boolean auto) {
    this.autoControlled = auto;
  }

  public void setAutoExtending(boolean auto) {
    this.autoExtending = auto;
  }

  public double getExtendAngle() {
    return extendAngle;
  }

  public void stop() {
    intakeIO.extend(0.0);
  }

  public void extend(double dutyCycle) {
    intakeIO.extend(dutyCycle);
  }

  public void enableBrakeMode() {
    intakeIO.enableBrakeMode();
  }

  public void enableCoastMode() {
    intakeIO.enableCoastMode();
  }
}
