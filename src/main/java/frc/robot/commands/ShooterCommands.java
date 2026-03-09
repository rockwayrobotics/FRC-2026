package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldRelativeAccel;
import frc.robot.util.FieldRelativeSpeed;
import frc.robot.util.GoalUtils;
import frc.robot.util.LinearInterpolationTable;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterCommands {
  private static LinearInterpolationTable m_timeTable = ShooterConstants.kShotTimesTable;
  private static LinearInterpolationTable m_hoodTable = HoodConstants.kHoodTable;
  private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;

  private static LoggedNetworkNumber flywheelSpeed =
      new LoggedNetworkNumber("Shooter/FlywheelSpeedSetter", 4250);
  private static LoggedNetworkNumber hoodAngle =
      new LoggedNetworkNumber("Shooter/HoodAngleSetter", 25);

  // 75" away from hub (front to front) at 4000 rpm
  public static Command testShoot(Shooter shooter, Hood hood) {
    return Commands.run(
        () -> {
          double rpm = MathUtil.clamp(flywheelSpeed.get(), 3000, 5000);
          double hoodDegrees = MathUtil.clamp(hoodAngle.get(), 5, 45);
          shooter.setVelocityFlywheel(rpm);
          hood.setPositionHood(Degrees.of(hoodDegrees));
        },
        shooter);
  }

  public static Command aimOnMove(
      Shooter shooter, Hood hood, Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
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
              Translation2d fieldRelativeShooterOffset =
                  ShooterConstants.kShooterOffset.rotateBy(robotPose.getRotation());
              Translation2d shooterLocation =
                  robotPose.getTranslation().plus(fieldRelativeShooterOffset);
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

                Translation2d toTestGoal = testGoalLocation.minus(shooterLocation);

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
                  movingGoalLocation.minus(shooterLocation).getDistance(new Translation2d());

              // Get linear velocity
              Translation2d linearVelocity =
                  DriveCommands.getLinearVelocityFromJoysticks(
                      xSupplier.getAsDouble(), ySupplier.getAsDouble());

              Rotation2d targetAngle = movingGoalLocation.minus(shooterLocation).getAngle();

              drive.setTargetAngle(targetAngle);
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
              hood.setPositionHood(hoodAngleAsAngle);

              // }

            },
            drive,
            shooter,
            hood)
        .finallyDo(
            () -> {
              shooter.stop();
              hood.stop();
            });
  }

  public static Command manualFlywheel(Shooter shooter, DoubleSupplier rpmSupplier) {
    return Commands.run(() -> shooter.setVelocityFlywheel(rpmSupplier.getAsDouble()), shooter);
  }

  public static Command manualHood(Hood hood, DoubleSupplier angleSupplier) {
    return Commands.run(() -> hood.setPositionHood(Degrees.of(angleSupplier.getAsDouble())), hood);
  }
}
