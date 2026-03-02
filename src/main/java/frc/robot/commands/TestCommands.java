package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class TestCommands {

  public static Command testShot(Shooter shooter, Indexer indexer) {
    final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    final NetworkTable table = ntInstance.getTable("ShooterTest");

    // The setters are commented out here so that Elastic can restore the previously set
    // values. If it doesn't work, then running the command once will more or less do the same
    // thing?
    final DoubleTopic flywheelRPMTopic = table.getDoubleTopic("FlywheelRPM");
    final DoublePublisher flywheelRPMPublisher = flywheelRPMTopic.publish();
    // flywheelRPMPublisher.set(0);
    final DoubleSubscriber testRPMSubscriber = flywheelRPMTopic.subscribe(0.0);

    final DoubleTopic augerRPMTopic = table.getDoubleTopic("AugerRPM");
    final DoublePublisher augerRPMPublisher = augerRPMTopic.publish();
    // augerRPMPublisher.set(0);
    final DoubleSubscriber testAugerSubscriber = augerRPMTopic.subscribe(0.0);

    final DoubleTopic kickerRPMTopic = table.getDoubleTopic("KickerRPM");
    final DoublePublisher kickerRPMPublisher = kickerRPMTopic.publish();
    // kickerRPMPublisher.set(0);
    final DoubleSubscriber testKickerSubscriber = kickerRPMTopic.subscribe(0.0);

    final DoubleTopic hoodAngleTopic = table.getDoubleTopic("HoodAngle");
    final DoublePublisher hoodAnglePublisher = hoodAngleTopic.publish();
    // hoodAnglePublisher.set(15);
    final DoubleSubscriber testHoodSubscriber = hoodAngleTopic.subscribe(15);

    var sequence =
        Commands.sequence(
                Commands.runOnce(
                    () -> {
                      double flywheelRPM = testRPMSubscriber.get();
                      flywheelRPM = MathUtil.clamp(flywheelRPM, 0, 5000);
                      flywheelRPMPublisher.set(flywheelRPM); // In case it was out of range
                      shooter.setVelocityFlywheel(flywheelRPM);

                      if (Robot.isSimulation()) {
                        // FIXME: Enable this when it's safe to move the hood with PID
                        double hoodPosition = testHoodSubscriber.get();
                        hoodPosition = MathUtil.clamp(hoodPosition, 15, 45);
                        hoodAnglePublisher.set(hoodPosition); // In case it was out of range
                        shooter.setPositionHood(Degrees.of(hoodPosition));
                      }
                    }),
                Commands.waitUntil(() -> shooter.atFlywheelSetpoint(100)),
                Commands.run(
                    () -> {
                      double augerRPM = testAugerSubscriber.get();
                      augerRPM = MathUtil.clamp(augerRPM, 0, 1000);
                      augerRPMPublisher.set(augerRPM); // In case it was out of range
                      indexer.setVelocityAugers(augerRPM);

                      double kickerRPM = testKickerSubscriber.get();
                      kickerRPM = MathUtil.clamp(kickerRPM, 0, 2500);
                      kickerRPMPublisher.set(kickerRPM); // In case it was out of range
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
