package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GoalUtils {
  public static Translation2d getHubLocation() {
    var field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    // Red alliance hub is at the y position of ID 10 and the x position of ID 8.
    Translation2d redHub =
        new Translation2d(
            field.getTagPose(8).get().toPose2d().getX(),
            field.getTagPose(10).get().toPose2d().getY());
    // Blue alliance hub is at the y position of ID 25 and the x position of ID 18.
    Translation2d blueHub =
        new Translation2d(
            field.getTagPose(18).get().toPose2d().getX(),
            field.getTagPose(25).get().toPose2d().getY());

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redHub;
    } else {
      return blueHub;
    }
  }
}
