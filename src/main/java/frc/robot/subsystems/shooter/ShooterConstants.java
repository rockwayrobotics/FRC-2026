package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class ShooterConstants {
  // Flywheel PIDV Constants
  public static final double FLYWHEEL_KP = 0.001;
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.05;
  public static final double FLYWHEEL_KV = 0.00117;
  // Hood PID Constants
  public static final double HOOD_KP = 0.033;
  public static final double HOOD_KI = 0.0;
  public static final double HOOD_KD = 0.0;

  // FIXME: KCOS wants zero degrees at horizontal which we currently don't have.
  public static final double HOOD_KS = 0; // 0.0013;
  public static final double HOOD_KCOS = 0; // .0004;
  public static final double HOOD_KCOSRATIO = 0; // 0.76744186046511627906976744186047;

  // FIXME: Check these are correct!
  public static final double HOOD_FORWARD_LIMIT = 45; // 45 degree hood angle (50.4 max)
  public static final double HOOD_REVERSE_LIMIT = 5; // 15 degree hood angle (2.4 min)
  public static final double HOOD_ENCODER_POSITION_CONVERSION_FACTOR = 360;
  public static final double FLYWHEEL_GEAR_RATIO = 0.5;

  public static final double ACCEL_COMP_FACTOR = 0.100; // in units of seconds
  public static final Translation2d kShooterOffset = new Translation2d(-0.2278, 0.04319);

  private static final Point2D[] kHoodPoints =
      new Point2D.Double[] {
        // (distance, hood angle)
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
        // (distance, flywheel RPM)
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
        // (distance,time)
        new Point2D.Double(80, 0.78),
        new Point2D.Double(130, 0.80),
        new Point2D.Double(190, 0.81),
        new Point2D.Double(240, 0.82),
        new Point2D.Double(280, 0.83)
      };

  public static final LinearInterpolationTable kShotTimesTable =
      new LinearInterpolationTable(kShotTimes);

  private static final Point2D[] kHoodAngles =
      new Point2D.Double[] {
        // (Hood Angle, Through Bore Encoder Angle)
        new Point2D.Double(45, HOOD_FORWARD_LIMIT), new Point2D.Double(15, HOOD_REVERSE_LIMIT)
      };

  public static final LinearInterpolationTable kHoodAnglesTable =
      new LinearInterpolationTable(kHoodAngles);
}
/*

minimum hood position:
0.34346
maximum hood position:
0.46948

minimum voltage before moving up:
v1 = 0.1884                 NEW v1 = 0.017
maximum voltage before moving down:
v2 = -0.12                 NEW v2 = -0.009

*/
