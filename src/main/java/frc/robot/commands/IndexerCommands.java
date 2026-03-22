package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.GameHubStatus;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerCommands {
  public static final double FLYWHEEL_RPM_TOLERANCE = 100;
  public static final double HEADING_TOLERANCE_DEGREES = 5;
  public static boolean isShooting = false;

  private static LoggedNetworkNumber augerDutyNumber =
      new LoggedNetworkNumber("Indexer/AugerDuty", IndexerConstants.FEED_DUTY_CYCLE);
  private static LoggedNetworkNumber kickerVelocityNumber =
      new LoggedNetworkNumber("Indexer/KickerRPM", KickerConstants.KICKER_FEED_RPM);

  public static Command feedShooterFancy(
      Indexer indexer, Kicker kicker, Shooter shooter, Drive drive) {
    return Commands.sequence(
            Commands.waitUntil(() -> shooter.atFlywheelSetpoint(FLYWHEEL_RPM_TOLERANCE)),
            Commands.run(
                () -> {
                  if (drive.pointedAtTargetAngle(HEADING_TOLERANCE_DEGREES)) {
                    if (GameHubStatus.isHubActive()) {
                      indexer.augersFeed();
                      kicker.setVelocityKicker(KickerConstants.KICKER_FEED_RPM);
                      isShooting = true;
                    }
                  } else {
                    indexer.stop();
                    kicker.stop();
                    isShooting = false;
                  }
                },
                indexer,
                kicker))
        .finallyDo(
            () -> {
              indexer.stop();
              kicker.stop();
            });
  }

  public static Command feedShooter(Indexer indexer, Kicker kicker) {
    return Commands.sequence(
            Commands.run(
                () -> {
                  // indexer.augersFeed();
                  // indexer.setVelocityKicker(IndexerConstants.KICKER_FEED_RPM);
                  double augerDuty = MathUtil.clamp(augerDutyNumber.get(), 0.1, 1.0);
                  double kickerRPM = MathUtil.clamp(kickerVelocityNumber.get(), 3000, 5000);
                  indexer.setAugers(augerDuty);
                  kicker.setVelocityKicker(kickerRPM);
                },
                indexer,
                kicker))
        .finallyDo(
            () -> {
              indexer.stop();
              kicker.stop();
            });
  }

  public static Command agitate(Indexer indexer, Kicker kicker) {
    return Commands.run(
        () -> {
          indexer.augersAgitate();
          kicker.setVelocityKicker(KickerConstants.KICKER_AGITATE_RPM);
        },
        indexer,
        kicker);
  }

  public static Command augersFeed(Indexer indexer) {
    return Commands.run(() -> indexer.augersFeed(), indexer);
  }

  public static Command augersReverse(Indexer indexer) {
    return Commands.run(() -> indexer.augersReverse(), indexer);
  }

  public static Command augersDutyCycle(Indexer indexer, double dutyCycle) {
    return Commands.run(() -> indexer.setAugers(dutyCycle), indexer);
  }

  public static Command toggleKickerVelocity(Kicker kicker, double rpm) {
    if (Math.abs(kicker.getRpmSetpoint() - rpm) < 50) {
      return Commands.run(() -> kicker.setVelocityKicker(0), kicker);
    }
    return Commands.run(() -> kicker.setVelocityKicker(rpm), kicker);
  }
}
