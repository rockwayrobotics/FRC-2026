// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera_front = "OV9782_Front";
  public static String camera_back = "OV9281_Back";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // 31/16" from left of frame perimeter
  // 20 1/2" above floor
  // 11 3/4" from center
  public static Transform3d robotToCameraFront =
      new Transform3d(
          0.30246, // was 298 mm
          0.400567, // was 330 mm
          0.526307, // was 520.7 mm
          new Rotation3d(0, 0, 0));
  //   Inches.of(11.75).in(Meters),
  //   Inches.of(27.5 * 0.5 - 0.7375).in(Meters),
  //   Inches.of(20.5).in(Meters),
  //  new Rotation3d(0.0, -10 * Math.PI / 180.0, Degrees.of(-5).in(Radians)));
  public static Transform3d robotToCameraBack =
      new Transform3d(
          -Inches.of(27.5 * 0.5).in(Meters) - 0.1014,
          -Inches.of(27.5 * 0.5).in(Meters) + 0.304,
          0.422,
          new Rotation3d(0, 0, Math.PI)); // + Degrees.of(1.060911690264227).in(Radians)));
  // -Inches.of(27.5 * 0.5).in(Meters) + 0.012, -0.042, 0.42, new Rotation3d(0, 0, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
