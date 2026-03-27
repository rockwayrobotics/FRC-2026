package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GoalUtils {
  private static AprilTagFieldLayout field = null;

  public static AprilTagFieldLayout getField() {
    if (field == null) {
      field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }
    return field;
  }

  public static Translation2d getHubLocation() {
    var field = getField();
    // Red alliance hub is at the y position of ID 10 and the x position of ID 5.
    Translation2d redHub =
        new Translation2d(
            field.getTagPose(5).get().toPose2d().getX(),
            field.getTagPose(10).get().toPose2d().getY());
    // Blue alliance hub is at the y position of ID 26 and the x position of ID 18.
    Translation2d blueHub =
        new Translation2d(
            field.getTagPose(18).get().toPose2d().getX(),
            field.getTagPose(26).get().toPose2d().getY());

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redHub;
    } else {
      return blueHub;
    }
  }

  public static Translation2d getLeftTarget() {
    var field = getField();
    double redX = field.getTagPose(16).get().toPose2d().getX() - 1;
    // (field.getTagPose(7).get().toPose2d().getX() + field.getTagPose(16).get().toPose2d().getX())
    //     / 2;
    double redY = (field.getTagPose(9).get().toPose2d().getY()) / 2 + 1;
    Translation2d redTarget = new Translation2d(redX, redY);

    double blueX = 1;
    // (field.getTagPose(26).get().toPose2d().getX()
    //         + field.getTagPose(28).get().toPose2d().getX())
    //     / 2;
    double blueY = field.getFieldWidth() - ((field.getTagPose(25).get().toPose2d().getY() / 2) - 1);
    Translation2d blueTarget = new Translation2d(blueX, blueY);

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redTarget;
    } else {
      return blueTarget;
    }
  }

  public static Translation2d getRightTarget() {
    var field = getField();
    double redX = field.getTagPose(16).get().toPose2d().getX() - 1;
    // (field.getTagPose(7).get().toPose2d().getX() + field.getTagPose(16).get().toPose2d().getX())
    //     / 2;
    double redY = field.getFieldWidth() - ((field.getTagPose(25).get().toPose2d().getY() / 2) - 1);
    Translation2d redTarget = new Translation2d(redX, redY);

    double blueX = 1;
    // (field.getTagPose(26).get().toPose2d().getX()
    //         + field.getTagPose(28).get().toPose2d().getX())
    //     / 2;
    double blueY = (field.getTagPose(9).get().toPose2d().getY()) / 2 + 1;
    Translation2d blueTarget = new Translation2d(blueX, blueY);

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return redTarget;
    } else {
      return blueTarget;
    }
  }
}
