package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intakeExtender.IntakeExtender;
import frc.robot.subsystems.intakeExtender.IntakeExtenderConstants;
import java.util.concurrent.atomic.AtomicInteger;

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

  public static Command extendManual(IntakeExtender intakeExtender, double dutyCycle) {
    return Commands.startRun(
        () -> {
          intakeExtender.setHoldingPosition(true);
        },
        () -> intakeExtender.extend(dutyCycle),
        intakeExtender);
  }

  public static Command intakeManual(Intake intake, double dutyCycle) {
    return Commands.run(() -> intake.intake(dutyCycle), intake);
  }

  public static Command intakeFancy(Intake intake, double dutyCycle) {
    AtomicInteger overCurrentCount = new AtomicInteger(0);
    AtomicInteger recoveryCountdown = new AtomicInteger(0);
    return Commands.run(
            () -> {
              int recoveryCountdownValue = recoveryCountdown.get();
              if (recoveryCountdownValue > 0) {
                recoveryCountdown.decrementAndGet();
                intake.intake(IntakeConstants.ROLLER_EJECT_DUTY_CYCLE);
              } else {
                if (intake.getOverCurrentAuto()) {
                  int overCurrentValue = overCurrentCount.incrementAndGet();
                  if (overCurrentValue > 10) {
                    recoveryCountdown.set(12); // 240 ms
                  }
                } else {
                  if (overCurrentCount.get() > 0) {
                    overCurrentCount.decrementAndGet();
                  }
                }
                intake.intake(dutyCycle);
              }
            },
            intake)
        .finallyDo(
            () -> {
              recoveryCountdown.set(0);
              overCurrentCount.set(0);
            });
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
