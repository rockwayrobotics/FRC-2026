package frc.robot.commands.shooterCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldRelativeAccel;
import frc.robot.util.FieldRelativeSpeed;
import frc.robot.util.GoalUtils;
import frc.robot.util.LinearInterpolationTable;
import java.util.function.DoubleSupplier;

public class ShootOnMove {
  private static LinearInterpolationTable m_timeTable = ShooterConstants.kTimeTable;
  private static LinearInterpolationTable m_hoodTable = ShooterConstants.kHoodTable;
  private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;

  public static Command run(
      Shooter shooter, Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveCommands.ANGLE_KP,
            0.0,
            DriveCommands.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {
              Pose2d robotPose = drive.getPose();
              Translation2d fieldRelativeShooterOffset = ShooterConstants.kShooterOffset.rotateBy(robotPose.getRotation());
              Translation2d shooterLocation = robotPose.getTranslation().plus(fieldRelativeShooterOffset);
              FieldRelativeSpeed robotVel = drive.getFieldRelativeSpeed();
              FieldRelativeAccel robotAccel = drive.getFieldRelativeAccel();

              Translation2d target = GoalUtils.getHubLocation();

              Translation2d robotToGoal = target.minus(shooterLocation);
              double dist = robotToGoal.getDistance(new Translation2d());

              double shotTime = m_timeTable.getOutput(dist);
              Translation2d movingGoalLocation = new Translation2d();
              for (int i = 0; i < 5; i++) {

                double virtualGoalX =
                    target.getX()
                        - shotTime
                            * (robotVel.vx + robotAccel.ax * ShooterConstants.ACCEL_COMP_FACTOR);
                double virtualGoalY =
                    target.getY()
                        - shotTime
                            * (robotVel.vy + robotAccel.ay * ShooterConstants.ACCEL_COMP_FACTOR);

                Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

                Translation2d toTestGoal = testGoalLocation.minus(drive.getPose().getTranslation());

                double newShotTime =
                    m_timeTable.getOutput(toTestGoal.getDistance(new Translation2d()));

                if (Math.abs(newShotTime - shotTime) <= 0.010) {
                  i = 4;
                }

                if (i == 4) {
                  movingGoalLocation = testGoalLocation;
                } else {
                  shotTime = newShotTime;
                }
              }

              double newDist =
                  movingGoalLocation
                      .minus(drive.getPose().getTranslation())
                      .getDistance(new Translation2d());

              // Get linear velocity
              Translation2d linearVelocity =
                  DriveCommands.getLinearVelocityFromJoysticks(
                      xSupplier.getAsDouble(), ySupplier.getAsDouble());

              Rotation2d targetAngle =
                  movingGoalLocation.minus(drive.getPose().getTranslation()).getAngle();

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), targetAngle.getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

              /*
               * // FIXME: This would allow tuning from dashboard:
               * if (SmartDashboard.getBoolean("Adjust Shot?", false)) {
               * // FIXME: This should be a speed
               * shooter.setVoltageShooter(m_rpmTable.getOutput(newDist) +
               * SmartDashboard.getNumber("SetShotAdjust", 0));
               * shooter.setVoltageHood(m_hoodTable.getOutput(newDist) +
               * SmartDashboard.getNumber("SetHoodAdjust", 0));
               * } else {
               */
              shooter.setVelocityFlywheel(m_rpmTable.getOutput(newDist));
              Angle hoodAngleAsAngle = Degrees.of(m_hoodTable.getOutput(newDist));
              shooter.setPositionHood(hoodAngleAsAngle);

              // }

            },
            drive,
            shooter)
        .finallyDo(
            () -> {
              shooter.stop();
            });
  }
}
