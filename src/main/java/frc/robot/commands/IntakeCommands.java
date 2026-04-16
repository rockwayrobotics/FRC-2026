package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intakeExtender.IntakeExtender;
import frc.robot.subsystems.intakeExtender.IntakeExtenderConstants;

public class IntakeCommands {
  public static Command extend(IntakeExtender intakeExtender) {
    return Commands.run(
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
    return Commands.run(
        () -> {
          if (intakeExtender.getExtendAngle() > IntakeExtenderConstants.RETRACT_LIMIT) {
            intakeExtender.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
          } else {
            intakeExtender.extend(0.0);
          }
        },
        intakeExtender);
  }

  public static Command trashCompact(IntakeExtender intakeExtender, Intake intake) {
    return Commands.parallel(
        Commands.run(
            () -> {
              intake.intake(IntakeConstants.TRASH_COMPACT_DUTY_CYCLE);
            }),
        new SequentialCommandGroup(
                Commands.run(
                        () -> {
                          intakeExtender.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
                        })
                    .until(
                        () ->
                            intakeExtender.getExtendAngle()
                                <= IntakeExtenderConstants.TRASH_COMPACT_RETRACT_LIMIT)
                    .withTimeout(1),
                Commands.run(
                        () -> {
                          intakeExtender.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
                        })
                    .until(
                        () ->
                            intakeExtender.getExtendAngle()
                                >= IntakeExtenderConstants.TRASH_COMPACT_EXTEND_LIMIT)
                    .withTimeout(1))
            .repeatedly());
  }

  public static Command trashCompactAuto(IntakeExtender intakeExtender, Intake intake) {
    return Commands.parallel(
        Commands.run(
            () -> {
              intake.intake(IntakeConstants.TRASH_COMPACT_DUTY_CYCLE);
            }),
        new SequentialCommandGroup(
                Commands.run(
                        () -> {
                          intakeExtender.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
                        })
                    .until(
                        () ->
                            intakeExtender.getExtendAngle()
                                <= IntakeExtenderConstants.TRASH_COMPACT_RETRACT_LIMIT)
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          intakeExtender.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
                        })
                    .until(
                        () ->
                            intakeExtender.getExtendAngle()
                                >= IntakeExtenderConstants.TRASH_COMPACT_EXTEND_LIMIT)
                    .withTimeout(0.5))
            .repeatedly());
  }

  public static Command extendManual(IntakeExtender intakeExtender, double dutyCycle) {
    return Commands.run(() -> intakeExtender.extend(dutyCycle), intakeExtender);
  }

  public static Command intakeManual(Intake intake, double dutyCycle) {
    return Commands.run(() -> intake.intake(dutyCycle), intake);
  }

  public static Command intakeManualWithRumble(
      Intake intake, double dutyCycle, CommandXboxController controller) {
    return Commands.run(
            () -> {
              intake.intake(dutyCycle);
              if (intake.getOverCurrent()) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
              }
            },
            intake)
        .finallyDo(
            () -> {
              controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            });
  }
}
