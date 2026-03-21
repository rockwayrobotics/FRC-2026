package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class ShooterConstants {
  public static final Translation2d kShooterOffset = new Translation2d(-0.2278, 0.04319);

  // Flywheel PIDV Constants
  public static final double FLYWHEEL_KP = 0.00067; //0.0008; // 0.001;
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.0167; //0.02;
  public static final double FLYWHEEL_KV = 0.00075; //0.0009;
  public static final double HOOD_KCOSRATIO = 0; // 0.76744186046511627906976744186047;

  public static final double FLYWHEEL_GEAR_RATIO = 0.6; // 18 / 30

  public static final double ACCEL_COMP_FACTOR = 0.100; // in units of seconds

  public static final double FLYWHEEL_MAX_RPM = 6666.67;

  // meters, RPM
  private static final Point2D[] kRPMPoints =
      new Point2D.Double[] {
        // (distance, flywheel RPM)
        // Old data - what we ran at Durham
        // new Point2D.Double(1.0, 2666.67), // cant get this close to hub
        // new Point2D.Double(1.5, 2666.67),
        // new Point2D.Double(2.25, 3041.67),
        // new Point2D.Double(3.0, 3333.33),
        // new Point2D.Double(3.46, 3333.33),
        // new Point2D.Double(4.0, 3583.33),
        // new Point2D.Double(4.25, 3583.33),
        // new Point2D.Double(4.94, 3916.67),

        new Point2D.Double(1.15, 2416.67), // 5 degrees

        //        new Point2D.Double(2.5, 2916.67), // 15 degrees
        new Point2D.Double(2.5, 2833.33), // 20 degrees
        //        new Point2D.Double(2.5, 2750), // 25 degrees
        //        new Point2D.Double(2.5, 2791.67), // 30 degrees - sketchy angle

        //        new Point2D.Double(3.15, 3250), // 15 degrees
        //        new Point2D.Double(3.15, 3083.33), // 20 degrees
        new Point2D.Double(3.15, 3000), // 25 degrees
        //        new Point2D.Double(3.15, 2916.67), // 30 degrees
        //        new Point2D.Double(3.15, 2833.33), // 35 degrees

        // close to trench
        //        new Point2D.Double(3.5, 3500), // 10 degrees High Spread
        //        new Point2D.Double(3.5, 3291.67), // 15 degrees
        //        new Point2D.Double(3.5,3208.33), // 20 Degrees
        // new Point2D.Double(3.5, 3187.5), // 22.5 Degrees
        // new Point2D.Double(3.5, 3125), // 25 Degrees
        new Point2D.Double(3.5, 3083.33), // 27.5 Degrees
        //        new Point2D.Double(3.5,3041.67), // 30 Degrees
        //        new Point2D.Double(3.5,2916.67), // 35 Degrees

        //        new Point2D.Double(4.00, 3333.33), // 20 degrees
        //        new Point2D.Double(4.01, 3312.5), // 25 degrees
        new Point2D.Double(4.01, 3250), // 30 degrees
        //        new Point2D.Double(4.01, 3208.33), // 35 degrees
        //        new Point2D.Double(4.01, 3208.33), // 40 degrees

        //        new Point2D.Double(5.36, 4125), //10 degrees
        //        new Point2D.Double(5.36, 4125), //15 degrees
        //        new Point2D.Double(5.36, 3958.33), //20 degrees
        // new Point2D.Double(5.36, 3875), // 25 degrees
        //        new Point2D.Double(5.36, 3833.33), //30 degrees
        new Point2D.Double(5.36, 3750), // 35 degrees
        //        new Point2D.Double(5.36, 3666.67), //40 degrees
        //        new Point2D.Double(5.36, 3458.33), //45 degrees

      };

  public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

  private static final Point2D[] kRPMGoalPoints = {
    // 45 degrees goal shots, measured distance from hub
    new Point2D.Double(4.5, 3333.33),
    new Point2D.Double(6.0, 3666.67),
    new Point2D.Double(8, 3875),
    new Point2D.Double(10.8, 5083.33),
    new Point2D.Double(12.13, 5500),
    new Point2D.Double(14, 6083.33),
  };

  public static final LinearInterpolationTable kRPMGoalTable =
      new LinearInterpolationTable(kRPMGoalPoints);

  // FIXME: Record these in meters and seconds
  // We couldn't figure out how to record these accurately
  // enough. Our videos vary wildly between 0.5 and 1 sec.
  private static final Point2D[] kShotTimes =
      new Point2D.Double[] {
        // (distance,time)
        new Point2D.Double(80, 0.78),
        new Point2D.Double(130, 0.80),
        new Point2D.Double(190, 0.81),
        new Point2D.Double(240, 0.82),
        new Point2D.Double(280, 0.83)
      };

  public static final LinearInterpolationTable kShotTimesTable =
      new LinearInterpolationTable(kShotTimes);
}
