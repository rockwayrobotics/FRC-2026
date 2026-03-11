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

  // meters, RPM
  private static final Point2D[] kRPMPoints =
      new Point2D.Double[] {
        // (distance, flywheel RPM)
        new Point2D.Double(2.44, 3900), // more data needed
        new Point2D.Double(3.09, 4100), // tower
        new Point2D.Double(3.21, 4175), // side tower
        new Point2D.Double(3.41, 4250), // trench
        new Point2D.Double(4.56, 4675), // corner
        // new Point2D.Double(35, 1500 + 10),
        // new Point2D.Double(55, 1860 + 10),
        // new Point2D.Double(80, 2000 + 10), //
        // new Point2D.Double(105, 2100 + 10), //
        // new Point2D.Double(130, 2170 + 20), //
        // new Point2D.Double(155, 2245 + 30), //
        // new Point2D.Double(165, 2380), //
        // new Point2D.Double(180, 2465 + 30), //
        // new Point2D.Double(205, 2670 + 30), //
        // new Point2D.Double(230, 2840 + 35), //
        // new Point2D.Double(255, 2980 + 40), //
        // new Point2D.Double(270, 3300), //
        // new Point2D.Double(280, 3350 + 60)
      };

  public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

  // FIXME: Record these in meters and seconds
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
