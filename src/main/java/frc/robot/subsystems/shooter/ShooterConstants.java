package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class ShooterConstants {
  public static final Translation2d kShooterOffset = new Translation2d(-0.2278, 0.04319);

  // Flywheel PIDV Constants
  public static final double FLYWHEEL_KP = 0.0008; // 0.001;
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.02;
  public static final double FLYWHEEL_KV = 0.0009;
  public static final double HOOD_KCOSRATIO = 0; // 0.76744186046511627906976744186047;

  public static final double FLYWHEEL_GEAR_RATIO = 0.5;

  public static final double ACCEL_COMP_FACTOR = 0.100; // in units of seconds

  public static final double FLYWHEEL_MAX_RPM = 8000;

  // meters, RPM
  private static final Point2D[] kRPMPoints =
      new Point2D.Double[] {
        // (distance, flywheel RPM)
        // Old data - what we ran at Durham
        // new Point2D.Double(1.0, 3200), // cant get this close to hub
        // new Point2D.Double(1.5, 3200),
        // new Point2D.Double(2.25, 3650),
        // new Point2D.Double(3.0, 4000),
        // new Point2D.Double(3.46, 4000),
        // new Point2D.Double(4.0, 4300),
        // new Point2D.Double(4.25, 4300),
        // new Point2D.Double(4.94, 4700),

        new Point2D.Double(1.15, 2900), // 5 degrees

        //        new Point2D.Double(2.5, 3500), // 15 degrees
        new Point2D.Double(2.5, 3400), // 20 degrees
        //        new Point2D.Double(2.5, 3300), // 25 degrees
        //        new Point2D.Double(2.5, 3350), // 30 degrees - sketchy angle

        //        new Point2D.Double(3.15, 3900), // 15 degrees
        //        new Point2D.Double(3.15, 3700), // 20 degrees
        new Point2D.Double(3.15, 3600), // 25 degrees
        //        new Point2D.Double(3.15, 3500), // 30 degrees
        //        new Point2D.Double(3.15, 3400), // 35 degrees

        // close to trench
        //        new Point2D.Double(3.5, 4200), // 10 degrees High Spread
        //        new Point2D.Double(3.5, 3950), // 15 degrees
        //        new Point2D.Double(3.5,3850), // 20 Degrees
        // new Point2D.Double(3.5, 3825), // 22.5 Degrees
        // new Point2D.Double(3.5, 3750), // 25 Degrees
        new Point2D.Double(3.5, 3700), // 27.5 Degrees
        //        new Point2D.Double(3.5,3650), // 30 Degrees
        //        new Point2D.Double(3.5,3500), // 35 Degrees

        //        new Point2D.Double(4.00, 4000), // 20 degrees
        //        new Point2D.Double(4.01, 3975), // 25 degrees
        new Point2D.Double(4.01, 3900), // 30 degrees
        //        new Point2D.Double(4.01, 3850), // 35 degrees
        //        new Point2D.Double(4.01, 3850), // 40 degrees

        //        new Point2D.Double(5.36, 4950), //10 degrees
        //        new Point2D.Double(5.36, 4950), //15 degrees
        //        new Point2D.Double(5.36, 4750), //20 degrees
        // new Point2D.Double(5.36, 4650), // 25 degrees
        //        new Point2D.Double(5.36, 4600), //30 degrees
        new Point2D.Double(5.36, 4500), // 35 degrees
        //        new Point2D.Double(5.36, 4400), //40 degrees
        //        new Point2D.Double(5.36, 4150), //45 degrees

      };

  public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

  private static final Point2D[] kRPMGoalPoints = {
    // 45 degrees goal shots, measured distance from hub
    new Point2D.Double(4.5, 4000),
    new Point2D.Double(6.0, 4400),
    new Point2D.Double(8, 4650),
    new Point2D.Double(10.8, 6100),
    new Point2D.Double(12.13, 6600),
    new Point2D.Double(14, 7300),
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
