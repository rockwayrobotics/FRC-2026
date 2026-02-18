package frc.robot.subsystems.shooter;

import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class ShooterConstants {
  // FIXME: These constants are entirely made up.
  public static final double HOOD_FORWARD_LIMIT = 0.0;
  public static final double HOOD_REVERSE_LIMIT = 30.0;

  public static final double ACCEL_COMP_FACTOR = 0.100; // in units of seconds

  private static final Point2D[] kHoodPoints =
      new Point2D.Double[] {
        // (ty-angle,distance)
        new Point2D.Double(35, 0.0),
        new Point2D.Double(55, 0.0),
        new Point2D.Double(80, 7.5), //
        new Point2D.Double(105, 16.5), //
        new Point2D.Double(130, 22.0), //
        new Point2D.Double(155, 25.5), //
        new Point2D.Double(165, 25.5), //
        new Point2D.Double(180, 27.5), //
        new Point2D.Double(205, 29.0), //
        new Point2D.Double(230, 33.0), //
        new Point2D.Double(255, 33.0), //
        new Point2D.Double(270, 33.5), //
        new Point2D.Double(280, 36.1)
      };

  public static final LinearInterpolationTable kHoodTable =
      new LinearInterpolationTable(kHoodPoints);

  private static final Point2D[] kRPMPoints =
      new Point2D.Double[] {
        // (ty-angle,distance)
        new Point2D.Double(35, 1500 + 10),
        new Point2D.Double(55, 1860 + 10),
        new Point2D.Double(80, 2000 + 10), //
        new Point2D.Double(105, 2100 + 10), //
        new Point2D.Double(130, 2170 + 20), //
        new Point2D.Double(155, 2245 + 30), //
        new Point2D.Double(165, 2380), //
        new Point2D.Double(180, 2465 + 30), //
        new Point2D.Double(205, 2670 + 30), //
        new Point2D.Double(230, 2840 + 35), //
        new Point2D.Double(255, 2980 + 40), //
        new Point2D.Double(270, 3300), //
        new Point2D.Double(280, 3350 + 60)
      };

  public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMPoints);

  private static final Point2D[] kShotTimes =
      new Point2D.Double[] {
        // (ty-angle,time)
        new Point2D.Double(80, 0.78),
        new Point2D.Double(130, 0.80),
        new Point2D.Double(190, 0.81),
        new Point2D.Double(240, 0.82),
        new Point2D.Double(280, 0.83)
      };

  public static final LinearInterpolationTable kTimeTable =
      new LinearInterpolationTable(kShotTimes);
}
