package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeExtender.IntakeExtender;
import frc.robot.subsystems.intakeExtender.IntakeExtenderConstants;
import java.util.concurrent.atomic.AtomicBoolean;

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

  public static Command trashCompact(IntakeExtender intakeExtender) {
    AtomicBoolean retracting = new AtomicBoolean(true);
    return Commands.run(
        () -> {
          if (retracting.get()) {
            if (intakeExtender.getExtendAngle()
                > IntakeExtenderConstants.TRASH_COMPACT_RETRACT_LIMIT) {
              intakeExtender.extend(IntakeExtenderConstants.RETRACT_DUTY_CYCLE);
            } else {
              retracting.set(false);
              intakeExtender.extend(0.0);
            }
          } else {
            if (intakeExtender.getExtendAngle()
                < IntakeExtenderConstants.TRASH_COMPACT_EXTEND_LIMIT) {
              intakeExtender.extend(IntakeExtenderConstants.EXTEND_DUTY_CYCLE);
            } else {
              retracting.set(true);
              intakeExtender.extend(0.0);
            }
          }
        },
        intakeExtender);
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
