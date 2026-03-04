package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intakeExtender.IntakeExtender;
import frc.robot.subsystems.intakeExtender.IntakeExtenderConstants;

public class IntakeCommands {
  public static Command autoIntake(IntakeExtender intakeExtender, Intake intake) {
    return Commands.run(
            () -> {
              if (intakeExtender.getExtendAngle() < IntakeExtenderConstants.EXTEND_LIMIT) {
                intakeExtender.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
              } else {
                intakeExtender.extend(0.0);
              }

              intake.intake(IntakeConstants.ROLLER_DUTY_CYCLE);
            },
            intakeExtender,
            intake)
        .withTimeout(2);
  }

  public static Command autoRetract(IntakeExtender intakeExtender, Intake intake) {
    return Commands.run(
            () -> {
              if (intakeExtender.getExtendAngle() > IntakeExtenderConstants.RETRACT_LIMIT) {
                intakeExtender.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
              } else {
                intakeExtender.extend(0.0);
              }

              intake.intake(0.0);
            },
            intakeExtender,
            intake)
        .withTimeout(2);
  }

  public static Command extend(IntakeExtender intakeExtender) {
    return Commands.runEnd(
        () -> {
          if (intakeExtender.getExtendAngle() < IntakeExtenderConstants.EXTEND_LIMIT) {
            intakeExtender.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
          } else {
            intakeExtender.extend(0.0);
          }
        },
        () -> intakeExtender.extend(0.0),
        intakeExtender);
  }

  public static Command retract(IntakeExtender intakeExtender) {
    return Commands.runEnd(
        () -> {
          if (intakeExtender.getExtendAngle() > IntakeExtenderConstants.RETRACT_LIMIT && intakeExtender.motorCurrentWithinLimit()) {
            intakeExtender.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
          } else {
            intakeExtender.extend(0.0);
          }
        },
        () -> intakeExtender.extend(0.0),
        intakeExtender);
  }

  public static Command extendManual(IntakeExtender intakeExtender, double dutyCycle) {
    return Commands.run(() -> intakeExtender.extend(dutyCycle), intakeExtender);
  }

  public static Command intakeManual(Intake intake, double dutyCycle) {
    return Commands.run(() -> intake.intake(dutyCycle), intake);
  }
}
