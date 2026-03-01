package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class TestCommands {

  public static Command testShot(Shooter shooter, Indexer indexer) {
    final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    final NetworkTable table = ntInstance.getTable("ShooterTest");

    var flywheelRPMTopic = table.getDoubleTopic("FlywheelRPM");
    flywheelRPMTopic.publish().set(0);
    final DoubleSubscriber testRPMSubscriber = flywheelRPMTopic.subscribe(0.0);

    var AugerRPMTopic = table.getDoubleTopic("AugerRPM");
    AugerRPMTopic.publish().set(0);
    final DoubleSubscriber testAugerSubscriber = AugerRPMTopic.subscribe(0.0);

    var KickerRPMTopic = table.getDoubleTopic("KickerRPM");
    KickerRPMTopic.publish().set(0);
    final DoubleSubscriber testKickerSubscriber = KickerRPMTopic.subscribe(0.0);

    // final DoubleSubscriber testHoodSubscriber = table.getDoubleTopic("HoodAngle").subscribe(15);

    var sequence =
        Commands.sequence(
                Commands.runOnce(
                    () -> {
                      double flywheelRPM = testRPMSubscriber.get();
                      flywheelRPM = MathUtil.clamp(flywheelRPM, 0, 5000);
                      shooter.setVelocityFlywheel(flywheelRPM);
                      // FIXME Add Hood Configs Properly.
                      // double hoodPosition = testHoodSubscriber.get();
                      // hoodPosition = MathUtil.clamp(hoodPosition, 15, 45);
                      // shooter.setPositionHood(hoodPosition);
                    }),
                Commands.waitUntil(() -> shooter.atFlywheelSetpoint(100)),
                Commands.runOnce(
                    () -> {
                      double augerRPM = testAugerSubscriber.get();
                      augerRPM = MathUtil.clamp(augerRPM, 0, 1000);
                      indexer.setVelocityAugers(augerRPM);

                      double kickerRPM = testKickerSubscriber.get();
                      kickerRPM = MathUtil.clamp(kickerRPM, 0, 2500);
                      indexer.setVelocityKicker(kickerRPM);
                    }))
            .finallyDo(
                () -> {
                  indexer.stop();
                  shooter.stop();
                });
    sequence.addRequirements(shooter, indexer);
    return sequence;
  }
  ;
}
