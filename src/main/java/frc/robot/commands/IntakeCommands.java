package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intakeExtender.IntakeExtender;
import frc.robot.subsystems.intakeExtender.IntakeExtenderConstants;

public class IntakeCommands {
  public static Command autoIntake(IntakeExtender intakeExtender, Intake intake) {
    return Commands.startRun(
        () -> {
          intakeExtender.setHoldingPosition(false);
          intakeExtender.unlockAutoRetract();
        },
        () -> {
          if (intakeExtender.getExtendAngle() < IntakeExtenderConstants.EXTEND_LIMIT) {
            intakeExtender.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
          } else {
            intakeExtender.extend(0.0);
          }

          intake.intake(IntakeConstants.ROLLER_DUTY_CYCLE);
        },
        intakeExtender,
        intake);
  }

  public static Command autoRetract(IntakeExtender intakeExtender) {
    return Commands.startRun(
        () -> {
          intakeExtender.unlockAutoRetract();
        },
        () -> {
          if (!intakeExtender.isHoldingPosition()
              && !intakeExtender.isAutoRetractBlocked()
              && intakeExtender.getExtendAngle() > IntakeExtenderConstants.RETRACT_LIMIT) {
            intakeExtender.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
          } else {
            intakeExtender.extend(0.0);
          }
        },
        intakeExtender);
  }

  public static Command extend(IntakeExtender intakeExtender) {
    return Commands.startRun(
        () -> {
          intakeExtender.setHoldingPosition(true);
        },
        () -> {
          if (intakeExtender.getExtendAngle() < IntakeExtenderConstants.EXTEND_LIMIT) {
            intakeExtender.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
          } else {
            intakeExtender.extend(0.0);
          }
        },
        intakeExtender);
  }

  public static Command retract(IntakeExtender intakeExtender) {
    // After unlocking holdingPosition, this will fall back to the default command
    // of autoRetract, which will actually retract if it can.
    return Commands.runOnce(
        () -> {
          intakeExtender.setHoldingPosition(false);
        },
        intakeExtender);
  }

  public static Command extendManual(IntakeExtender intakeExtender, double dutyCycle) {
    return Commands.startRun(
        () -> {
          intakeExtender.setHoldingPosition(true);
        },
        () -> intakeExtender.extend(dutyCycle),
        intakeExtender);
  }

  public static Command ejectBalls(IntakeExtender intakeExtender, Intake intake) {
    return Commands.run(
            () -> {
              // This is potentially dangerous because we're just saying
              // we can retract even if we might not be able to.
              intakeExtender.setHoldingPosition(false);
              intakeExtender.unlockAutoRetract();
              intake.intake(-IntakeConstants.ROLLER_EJECT_DUTY_CYCLE);
            },
            intake)
        .withTimeout(3.0);
  }

  public static Command intakeManual(Intake intake, DoubleSupplier dutyCycle) {
    return Commands.run(() -> intake.intake(dutyCycle.getAsDouble()), intake);
  }
}
