package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.GameHubStatus;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerCommands {
  public static final double FLYWHEEL_RPM_TOLERANCE = 100;
  public static final double HEADING_TOLERANCE_DEGREES = 5;
  public static boolean isShooting = false;

  private static LoggedNetworkNumber augerDutyNumber =
      new LoggedNetworkNumber("Indexer/AugerDuty", 0.6);
  private static LoggedNetworkNumber kickerVelocityNumber =
      new LoggedNetworkNumber("Indexer/KickerRPM", 3500);

  public static Command feedShooterFancy(Indexer indexer, Shooter shooter, Drive drive) {
    return Commands.sequence(
            Commands.waitUntil(() -> shooter.atFlywheelSetpoint(FLYWHEEL_RPM_TOLERANCE)),
            Commands.run(
                () -> {
                  if (drive.pointedAtTargetAngle(HEADING_TOLERANCE_DEGREES)) {
                    if (GameHubStatus.isHubActive()) {
                      indexer.augersFeed();
                      indexer.setVelocityKicker(IndexerConstants.KICKER_FEED_RPM);
                      isShooting = true;
                    }
                  } else {
                    indexer.stop();
                    isShooting = false;
                  }
                },
                indexer))
        .finallyDo(
            () -> {
              indexer.stop();
            });
  }

  public static Command feedShooter(Indexer indexer) {
    return Commands.sequence(
            Commands.run(
                () -> {
                  // indexer.augersFeed();
                  // indexer.setVelocityKicker(IndexerConstants.KICKER_FEED_RPM);
                  double augerDuty = MathUtil.clamp(augerDutyNumber.get(), 0.1, 0.6);
                  double kickerRPM = MathUtil.clamp(kickerVelocityNumber.get(), 3000, 5000);
                  indexer.setAugers(augerDuty);
                  indexer.setVelocityKicker(kickerRPM);
                },
                indexer))
        .finallyDo(
            () -> {
              indexer.stop();
            });
  }

  public static Command agitate(Indexer indexer) {
    return Commands.run(
        () -> {
          indexer.augersFeed();
          indexer.setVelocityKicker(IndexerConstants.KICKER_AGITATE_RPM);
        });
  }

  public static Command unjam(Indexer indexer) {
    return Commands.run(
        () -> {
          indexer.augersReverse();
          indexer.setVelocityKicker(IndexerConstants.KICKER_AGITATE_RPM);
        });
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

  public static Command kickerVelocity(Indexer indexer, double rpm) {
    return Commands.run(() -> indexer.setVelocityKicker(rpm), indexer);
  }
}
