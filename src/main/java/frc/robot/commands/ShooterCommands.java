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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldRelativeAccel;
import frc.robot.util.FieldRelativeSpeed;
import frc.robot.util.GoalUtils;
import frc.robot.util.LinearInterpolationTable;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterCommands {
  private static LinearInterpolationTable m_timeTable = ShooterConstants.kShotTimesTable;
  private static LinearInterpolationTable m_hoodTable = HoodConstants.kHoodTable;
  private static LinearInterpolationTable m_rpmTable = ShooterConstants.kRPMTable;

  private static LoggedNetworkNumber flywheelSpeed =
      new LoggedNetworkNumber("Shooter/FlywheelSpeedSetter", 4800);
  private static LoggedNetworkNumber hoodAngle =
      new LoggedNetworkNumber("Shooter/HoodAngleSetter", 45);

  private static LoggedNetworkNumber flywheelDumpSpeed =
      new LoggedNetworkNumber("Shooter/DumpFlywheelSpeed", 2500);
  private static LoggedNetworkNumber hoodDumpAngle =
      new LoggedNetworkNumber("Shooter/DumpHoodAngle", 45);

  private static LoggedNetworkNumber trenchSetpointFlywheel =
      new LoggedNetworkNumber("Setpoints/TrenchFlywheel", 4350);
  private static LoggedNetworkNumber trenchSetpointHood =
      new LoggedNetworkNumber("Setpoints/TrenchHood", 20);
  private static LoggedNetworkNumber towerSetpointFlywheel =
      new LoggedNetworkNumber("Setpoints/TowerFlywheel", 4100);
  private static LoggedNetworkNumber towerSetpointHood =
      new LoggedNetworkNumber("Setpoints/TowerHood", 22);
  private static LoggedNetworkNumber sideTowerSetpointFlywheel =
      new LoggedNetworkNumber("Setpoints/SideTowerFlywheel", 4275);
  private static LoggedNetworkNumber sideTowerSetpointHood =
      new LoggedNetworkNumber("Setpoints/SideTowerHood", 20);
  private static LoggedNetworkNumber cornerSetpointFlywheel =
      new LoggedNetworkNumber("Setpoints/CornerFlywheel", 4675);
  private static LoggedNetworkNumber cornerSetpointHood =
      new LoggedNetworkNumber("Setpoints/CornerHood", 25);

  private static LoggedNetworkNumber flywheelScale =
      new LoggedNetworkNumber("Shooter/FlywheelOperatorScale", 8);
  private static LoggedNetworkNumber hoodScale =
      new LoggedNetworkNumber("Shooter/HoodOperatorScale", 0.4);

  // 75" away from hub (front to front) at 4000 rpm
  public static Command testShoot(Shooter shooter, Hood hood) {
    return Commands.run(
        () -> {
          double rpm = MathUtil.clamp(flywheelSpeed.get(), 3000, ShooterConstants.FLYWHEEL_MAX_RPM);
          double hoodDegrees =
              MathUtil.clamp(
                  hoodAngle.get(),
                  HoodConstants.HOOD_REVERSE_LIMIT,
                  HoodConstants.HOOD_FORWARD_LIMIT);
          shooter.setOperatorOverride(false);
          shooter.setVelocityFlywheel(rpm);
          hood.setOperatorOverride(false);
          hood.setPositionHood(Degrees.of(hoodDegrees));
        },
        shooter);
  }

  public static Command definitiveShoot(
      Shooter shooter, Hood hood, double flywheel, double hoodangle) {
    return Commands.runOnce(
        () -> {
          double rpm = MathUtil.clamp(flywheel, 2000, ShooterConstants.FLYWHEEL_MAX_RPM);
          double hoodDegrees =
              MathUtil.clamp(
                  hoodangle, HoodConstants.HOOD_REVERSE_LIMIT, HoodConstants.HOOD_FORWARD_LIMIT);
          shooter.setOperatorOverride(false);
          shooter.setVelocityFlywheel(rpm);
          hood.setOperatorOverride(false);
          hood.setPositionHood(Degrees.of(hoodDegrees));
        },
        shooter);
  }

  public static Command dumpShort(Shooter shooter, Hood hood) {
    return Commands.runOnce(
        () -> {
          double rpm =
              MathUtil.clamp(flywheelDumpSpeed.get(), 3000, ShooterConstants.FLYWHEEL_MAX_RPM);
          double hoodDegrees =
              MathUtil.clamp(
                  hoodDumpAngle.get(),
                  HoodConstants.HOOD_REVERSE_LIMIT,
                  HoodConstants.HOOD_FORWARD_LIMIT);
          shooter.setOperatorOverride(false);
          shooter.setVelocityFlywheel(rpm);
          hood.setOperatorOverride(false);
          hood.setPositionHood(Degrees.of(hoodDegrees));
        },
        shooter);
  }

  public static Command setupHubShot(
      Shooter shooter,
      Hood hood,
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier slowModeSupplier,
      BooleanSupplier ignoreSlewLimitSupplier) {
    ProfiledPIDController angleController = DriveCommands.getRotatePIDController();
    Command result =
        Commands.parallel(
            // Turn until facing hub
            Commands.startRun(
                    () -> {
                      angleController.reset(
                          MathUtil.angleModulus(drive.getPose().getRotation().getRadians()));
                    },
                    () -> {
                      Rotation2d targetAngle =
                          GoalUtils.getHubLocation()
                              .minus(drive.getPose().getTranslation())
                              .getAngle();

                      // Calculate angular speed
                      double omega =
                          angleController.calculate(
                              drive.getRotation().getRadians(), targetAngle.getRadians());

                      ChassisSpeeds speeds = null;
                      if (DriverStation.isAutonomous()) {
                        speeds = new ChassisSpeeds(0, 0, omega);
                      } else {
                        double slowModeMultiplier =
                            drive
                                .getSlowModeSlewRateLimiter()
                                .calculate(slowModeSupplier.getAsBoolean() ? 0.5 : 1.0);

                        // Get linear velocity
                        Translation2d linearVelocity =
                            DriveCommands.getLinearVelocityFromJoysticks(
                                    xSupplier.getAsDouble(), ySupplier.getAsDouble())
                                .times(slowModeMultiplier);

                        // Do NOT Square rotation value for more precise control in auto
                        // but dampen rotation during teleop because the controller is held down.
                        if (!DriverStation.isAutonomous()) {
                          omega = Math.copySign(omega * omega, omega) * slowModeMultiplier;
                        }

                        Translation2d targetVelocity =
                            linearVelocity.times(drive.getMaxLinearSpeedMetersPerSec());
                        double targetSpeed = targetVelocity.getNorm();
                        double limitedSpeed = targetSpeed;
                        if (!ignoreSlewLimitSupplier.getAsBoolean()) {
                          limitedSpeed = drive.getDriveSlewRateLimiter().calculate(targetSpeed);
                        }
                        double xSpeed =
                            targetSpeed < 0.001
                                ? 0.0
                                : targetVelocity.getX() * limitedSpeed / targetSpeed;
                        double ySpeed =
                            targetSpeed < 0.001
                                ? 0.0
                                : targetVelocity.getY() * limitedSpeed / targetSpeed;

                        // Convert to field relative speeds & send command
                        speeds =
                            new ChassisSpeeds(
                                xSpeed, ySpeed, omega * drive.getMaxAngularSpeedRadPerSec());
                      }

                      boolean isFlipped =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;

                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              speeds,
                              isFlipped
                                  ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                  : drive.getRotation()));
                    })
                .until(() -> DriverStation.isAutonomous() && angleController.atGoal())
                .andThen(Commands.runOnce(() -> drive.stop(), drive)),

            // Spin up flywheel based on table
            // Raise hood based on table
            Commands.run(
                    () -> {
                      double distance =
                          GoalUtils.getHubLocation().getDistance(drive.getPose().getTranslation());
                      double rpm = ShooterConstants.kRPMTable.getOutput(distance);
                      double hoodAngle = HoodConstants.kHoodTable.getOutput(distance);
                      shooter.setOperatorOverride(false);
                      shooter.setVelocityFlywheel(rpm);
                      hood.setOperatorOverride(false);
                      hood.setPositionHood(Degrees.of(hoodAngle));
                    })
                .until(() -> DriverStation.isAutonomous() && shooter.atFlywheelSetpoint(100)));
    result.addRequirements(shooter, hood, drive);
    return result;
  }

  public static Command hubShotWithoutAlign(
      Shooter shooter, Hood hood, Drive drive, CommandXboxController controller) {
    return Commands.run(
            () -> {
              double distance =
                  GoalUtils.getHubLocation().getDistance(drive.getPose().getTranslation());
              Logger.recordOutput("Shooter/TargetShotDistance", distance);
              double rpm = ShooterConstants.kRPMTable.getOutput(distance);
              double hoodAngle = HoodConstants.kHoodTable.getOutput(distance);
              shooter.setOperatorOverride(false);
              shooter.setVelocityFlywheel(rpm);
              hood.setOperatorOverride(false);
              hood.setPositionHood(Degrees.of(hoodAngle));
              if (shooter.atFlywheelSetpoint(100)) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
              }
            },
            shooter,
            hood)
        .finallyDo(
            () -> {
              controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            });
  }

  public static Command targetShotWithoutAlign(
      Shooter shooter,
      Hood hood,
      Drive drive,
      CommandXboxController controller,
      Supplier<Translation2d> targetSupplier) {
    return Commands.run(
            () -> {
              double distance = targetSupplier.get().getDistance(drive.getPose().getTranslation());
              Logger.recordOutput("Shooter/TargetShotDistance", distance);
              double rpm = ShooterConstants.kRPMGoalTable.getOutput(distance);
              double hoodAngle = HoodConstants.kHoodGoalTable.getOutput(distance);
              shooter.setOperatorOverride(false);
              shooter.setVelocityFlywheel(rpm);
              hood.setOperatorOverride(false);
              hood.setPositionHood(Degrees.of(hoodAngle));
              if (shooter.atFlywheelSetpoint(100)) {
                controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
              }
            },
            shooter,
            hood)
        .finallyDo(
            () -> {
              controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            });
  }

  public static Command setupGoalShot(
      Shooter shooter,
      Hood hood,
      Drive drive,
      Supplier<Translation2d> goalSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      BooleanSupplier slowModeSupplier,
      BooleanSupplier ignoreSlewLimitSupplier) {
    ProfiledPIDController angleController = DriveCommands.getRotatePIDController();
    Command result =
        Commands.parallel(
            // Turn until facing goal
            Commands.startRun(
                    () -> {
                      angleController.reset(
                          MathUtil.angleModulus(drive.getPose().getRotation().getRadians()));
                    },
                    () -> {
                      Rotation2d targetAngle =
                          goalSupplier.get().minus(drive.getPose().getTranslation()).getAngle();
                      Logger.recordOutput("SetupGoal/Goal", goalSupplier.get());
                      Logger.recordOutput("SetupGoal/TargetAngle", targetAngle);

                      // Calculate angular speed
                      double omega =
                          angleController.calculate(
                              drive.getRotation().getRadians(), targetAngle.getRadians());

                      ChassisSpeeds speeds = null;
                      if (DriverStation.isAutonomous()) {
                        speeds = new ChassisSpeeds(0, 0, omega);
                      } else {
                        double slowModeMultiplier =
                            drive
                                .getSlowModeSlewRateLimiter()
                                .calculate(slowModeSupplier.getAsBoolean() ? 0.5 : 1.0);

                        // Get linear velocity
                        Translation2d linearVelocity =
                            DriveCommands.getLinearVelocityFromJoysticks(
                                    xSupplier.getAsDouble(), ySupplier.getAsDouble())
                                .times(slowModeMultiplier);

                        // Do NOT Square rotation value for more precise control
                        // omega = Math.copySign(omega * omega, omega) * slowModeMultiplier;
                        Translation2d targetVelocity =
                            linearVelocity.times(drive.getMaxLinearSpeedMetersPerSec());
                        double targetSpeed = targetVelocity.getNorm();
                        double limitedSpeed = targetSpeed;
                        if (!ignoreSlewLimitSupplier.getAsBoolean()) {
                          limitedSpeed = drive.getDriveSlewRateLimiter().calculate(targetSpeed);
                        }
                        double xSpeed =
                            targetSpeed < 0.001
                                ? 0.0
                                : targetVelocity.getX() * limitedSpeed / targetSpeed;
                        double ySpeed =
                            targetSpeed < 0.001
                                ? 0.0
                                : targetVelocity.getY() * limitedSpeed / targetSpeed;

                        // Convert to field relative speeds & send command
                        speeds =
                            new ChassisSpeeds(
                                xSpeed, ySpeed, omega * drive.getMaxAngularSpeedRadPerSec());
                      }

                      boolean isFlipped =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;

                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              speeds,
                              isFlipped
                                  ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                  : drive.getRotation()));
                    })
                .until(() -> DriverStation.isAutonomous() && angleController.atGoal())
                .andThen(Commands.runOnce(() -> drive.stop(), drive)),

            // Spin up flywheel based on table
            // Raise hood based on table
            Commands.run(
                    () -> {
                      // WARNING: Disabled table for goal shot because we don't have data

                      // double distance =
                      //
                      // GoalUtils.getHubLocation().getDistance(drive.getPose().getTranslation());
                      // double rpm = ShooterConstants.kRPMTable.getOutput(distance);
                      // double hoodAngle = HoodConstants.kHoodTable.getOutput(distance);
                      // shooter.setVelocityFlywheel(rpm);
                      // hood.setPositionHood(Degrees.of(hoodAngle));

                      double rpm =
                          MathUtil.clamp(
                              flywheelSpeed.get(), 3000, ShooterConstants.FLYWHEEL_MAX_RPM);
                      double hoodDegrees =
                          MathUtil.clamp(
                              hoodAngle.get(),
                              HoodConstants.HOOD_REVERSE_LIMIT,
                              HoodConstants.HOOD_FORWARD_LIMIT);
                      shooter.setOperatorOverride(false);
                      shooter.setVelocityFlywheel(rpm);
                      hood.setOperatorOverride(false);
                      hood.setPositionHood(Degrees.of(hoodDegrees));
                    })
                .until(() -> DriverStation.isAutonomous() && shooter.atFlywheelSetpoint(100)));
    result.addRequirements(shooter, hood, drive);
    return result;
  }

  /**
   * Spins up flywheel to setpoint, stores desired hood angle but does not command any movement on
   * it.
   *
   * @param shooter IS a requirement
   * @param hood is NOT a requirement here, it just sets it up for triggering
   * @return
   */
  private static Command setpointShoot(
      Shooter shooter, Hood hood, DoubleSupplier hoodDegrees, DoubleSupplier rpmSupplier) {
    return Commands.startRun(
        () -> {
          Angle hoodAngle = Degrees.of(MathUtil.clamp(hoodDegrees.getAsDouble(), 5, 45));
          hood.setDeferredSetpoint(hoodAngle);
          shooter.setOperatorOverride(false);
        },
        () -> {
          shooter.setVelocityFlywheel(MathUtil.clamp(rpmSupplier.getAsDouble(), 3000, 5000));
        },
        shooter);
  }

  public static Command trenchSetpointShoot(Shooter shooter, Hood hood) {
    return setpointShoot(
        shooter, hood, () -> trenchSetpointHood.get(), () -> trenchSetpointFlywheel.get());
  }

  public static Command towerSetpointShoot(Shooter shooter, Hood hood) {
    return setpointShoot(
        shooter, hood, () -> towerSetpointHood.get(), () -> towerSetpointFlywheel.get());
  }

  public static Command sideTowerSetpointShoot(Shooter shooter, Hood hood) {
    return setpointShoot(
        shooter, hood, () -> sideTowerSetpointHood.get(), () -> sideTowerSetpointFlywheel.get());
  }

  public static Command cornerSetpointShoot(Shooter shooter, Hood hood) {
    return setpointShoot(
        shooter, hood, () -> cornerSetpointHood.get(), () -> cornerSetpointFlywheel.get());
  }

  public static Command activateDeferredHood(Hood hood) {
    return Commands.run(
        () -> {
          hood.setOperatorOverride(false);
          hood.setPositionHood(hood.getDeferredSetpoint());
        },
        hood);
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
              shooter.setOperatorOverride(false);
              shooter.setVelocityFlywheel(m_rpmTable.getOutput(newDist));
              Angle hoodAngleAsAngle = Degrees.of(m_hoodTable.getOutput(newDist));
              hood.setOperatorOverride(false);
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
    return Commands.run(
        () -> {
          double rpmChange = rpmSupplier.getAsDouble();
          if (shooter.isOperatorOverriding()) {
            double currentSpeed = Math.max(shooter.getFlywheelRPMSetpoint(), 2000);
            double newSpeed =
                Math.min(
                    ShooterConstants.FLYWHEEL_MAX_RPM,
                    currentSpeed + flywheelScale.getAsDouble() * rpmChange);
            if (newSpeed < 2000 && rpmChange < 0) {
              shooter.stop();
              shooter.setOperatorOverride(false);
            } else {
              shooter.setVelocityFlywheel(newSpeed);
            }
          }

          // Also if flywheel setpoints are ever set, then disable hold state
        },
        shooter);
  }

  public static Command manualHood(Hood hood, DoubleSupplier angleSupplier) {
    return Commands.run(
        () -> {
          double angleChange = angleSupplier.getAsDouble();
          if (hood.isOperatorOverriding()) {
            double currentAngle = hood.getHoodSetpoint().in(Degrees);
            double newAngle =
                MathUtil.clamp(
                    currentAngle + hoodScale.getAsDouble() * angleChange,
                    HoodConstants.HOOD_REVERSE_LIMIT,
                    HoodConstants.HOOD_FORWARD_LIMIT);
            hood.setPositionHood(Degrees.of(newAngle));
          }

          // Also if hood setpoints are ever set, then disable hold state
        },
        hood);
  }

  public static Command spinUp(Shooter shooter, Drive drive) {
    return Commands.run(
        () -> {
          var field = GoalUtils.getField();
          var pose = drive.getPose();
          double distance = 0;
          LinearInterpolationTable table = ShooterConstants.kRPMGoalTable;
          if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            // Red alliance
            // First check if we are close to our alliance zone, if so we want the shoot at hub
            // speed
            if (pose.getX() >= field.getTagPose(3).get().toPose2d().getX()) {
              distance = GoalUtils.getHubLocation().getDistance(pose.getTranslation());
              table = ShooterConstants.kRPMTable;
            } else if (pose.getY() > field.getFieldWidth() / 2) {
              // Then assume we are aiming for the left or right target based on our y-position
              distance = GoalUtils.getRightTarget().getDistance(pose.getTranslation());
            } else {
              distance = GoalUtils.getLeftTarget().getDistance(pose.getTranslation());
            }
          } else {
            // Blue alliance
            // First check if we are close to our alliance zone, if so we want the shoot at hub
            // speed
            if (pose.getX() <= field.getTagPose(25).get().toPose2d().getX()) {
              distance = GoalUtils.getHubLocation().getDistance(pose.getTranslation());
              table = ShooterConstants.kRPMTable;
            } else if (pose.getY() > field.getFieldWidth() / 2) {
              // Then assume we are aiming for the left or right target based on our y-position
              distance = GoalUtils.getLeftTarget().getDistance(pose.getTranslation());
            } else {
              distance = GoalUtils.getRightTarget().getDistance(pose.getTranslation());
            }
          }
          shooter.setOperatorOverride(false);
          shooter.setVelocityFlywheel(table.getOutput(distance));
        },
        shooter);
  }

  private static final LoggedNetworkNumber flywheelKp = new LoggedNetworkNumber("Flywheel/kp", 0);
  private static final LoggedNetworkNumber flywheelKi = new LoggedNetworkNumber("Flywheel/ki", 0);
  private static final LoggedNetworkNumber flywheelKd = new LoggedNetworkNumber("Flywheel/kd", 0);
  private static final LoggedNetworkNumber flywheelKv =
      new LoggedNetworkNumber("Flywheel/kv", 0.00117);

  public static Command configureLeader(Shooter shooter) {
    return Commands.runOnce(
        () -> {
          double kp = flywheelKp.getAsDouble();
          double ki = flywheelKi.getAsDouble();
          double kd = flywheelKd.getAsDouble();
          double kv = flywheelKv.getAsDouble();

          shooter.configureLeader(kp, ki, kd, kv);
        },
        shooter);
  }
}
