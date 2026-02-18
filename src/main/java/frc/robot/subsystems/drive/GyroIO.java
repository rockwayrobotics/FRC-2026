// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double velocityX = 0.0;
    public double velocityY = 0.0;
    public double velocityZ = 0.0;
    public double robotCentricVelocityX = 0.0;
    public double robotCentricVelocityY = 0.0;
    public double robotCentricVelocityZ = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
