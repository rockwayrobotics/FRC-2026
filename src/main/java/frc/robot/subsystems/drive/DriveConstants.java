// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CAN;
import lombok.Builder;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(22.50);
  public static final double wheelBase = Units.inchesToMeters(22.50);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // fl 2
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // fr 3
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // bl 1
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // br 4(0)
      };

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction =
      50 * 17 * 45 / 14 / 27 / 15; // L2 gearing for SDS Mk4i
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.09485;
  public static final double driveKv = 0.2375;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 150 / 7;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.5;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final SwerveModuleConfig[] swerveModuleConfigsComp = {
    // FL
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.FL_DRIVE_COMP)
        .turnMotorId(CAN.FL_TURN_COMP)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRadians(0)) // 1.439 + Math.PI / 4))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.FR_DRIVE_COMP)
        .turnMotorId(CAN.FR_TURN_COMP)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(0)) // -3.127 - Math.PI / 4))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.BL_DRIVE_COMP)
        .turnMotorId(CAN.BL_TURN_COMP)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(0)) // 2.651 + 3 * Math.PI / 4))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.BR_DRIVE_COMP)
        .turnMotorId(CAN.BR_TURN_COMP)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(0)) // -1.94 - 3 * Math.PI / 4))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final SwerveModuleConfig[] swerveModuleConfigsDev = {
    // FL
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.FL_DRIVE_DEV)
        .turnMotorId(CAN.FL_TURN_DEV)
        .encoderChannel(0) // 2
        // .encoderOffset(Rotation2d.fromRadians(0))
        .encoderOffset(Rotation2d.fromRadians(1.451 + Math.PI / 4)) // Saturday
        // .encoderOffset(Rotation2d.fromRadians(1.431 + Math.PI / 4)) // Friday night
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.FR_DRIVE_DEV)
        .turnMotorId(CAN.FR_TURN_DEV)
        .encoderChannel(1)
        // .encoderOffset(Rotation2d.fromRadians(0))
        .encoderOffset(Rotation2d.fromRadians(0.5425 - Math.PI / 4))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.BL_DRIVE_DEV)
        .turnMotorId(CAN.BL_TURN_DEV)
        .encoderChannel(3)
        // .encoderOffset(Rotation2d.fromRadians(0))
        .encoderOffset(Rotation2d.fromRadians(-3.0377 + 3 * Math.PI / 4))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    SwerveModuleConfig.builder()
        .driveMotorId(CAN.BR_DRIVE_DEV)
        .turnMotorId(CAN.BR_TURN_DEV)
        .encoderChannel(2)
        // .encoderOffset(Rotation2d.fromRadians(0))
        .encoderOffset(Rotation2d.fromRadians(0.815 - 3 * Math.PI / 4)) // Saturday
        // .encoderOffset(Rotation2d.fromRadians(0.792 - 3 * Math.PI / 4)) // Friday night
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  @Builder
  public record SwerveModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
