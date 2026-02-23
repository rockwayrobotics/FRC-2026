package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.GameHubStatus;

public class IndexerCommands {
  public static final double FLYWHEEL_RPM_TOLERANCE = 100;
  public static final double HEADING_TOLERANCE_DEGREES = 5;
  public static final double KICKER_LOAD_RPM = 2000;

  public static Command feedShooter(Indexer indexer, Shooter shooter, Drive drive) {
    return Commands.sequence(
            Commands.waitUntil(() -> shooter.atFlywheelSetpoint(FLYWHEEL_RPM_TOLERANCE)),
            Commands.run(
                () -> {
                  if (drive.pointedAtTargetAngle(HEADING_TOLERANCE_DEGREES)){
                    if (GameHubStatus.isHubActive()) { 
                      indexer.augersFeed();
                      indexer.setVelocityKicker(KICKER_LOAD_RPM);
                    }
                  } else {
                    indexer.stop();
                  }
                },
                indexer))
        .finallyDo(
            () -> {
              indexer.stop();
            });
  }
}
