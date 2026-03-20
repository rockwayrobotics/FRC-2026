package frc.robot.subsystems.hood;

import frc.robot.util.LinearInterpolationTable;
import java.awt.geom.Point2D;

public class HoodConstants {
  // Hood PID Constants
  public static final double HOOD_KP = 0.033;
  public static final double HOOD_KI = 0.0;
  public static final double HOOD_KD = 0.0;

  // FIXME: KCOS wants zero degrees at horizontal which we currently don't have.
  public static final double HOOD_KS = 0; // 0.0013;
  public static final double HOOD_KCOS = 0; // .0004;
  public static final double HOOD_KCOSRATIO = 0; // 0.76744186046511627906976744186047;

  // FIXME: Check these are correct!
  public static final double HOOD_REST_POINT = 20;
  public static final double HOOD_FORWARD_LIMIT = 46;
  public static final double HOOD_SAFE_FORWARD_LIMIT = 45;
  public static final double HOOD_SAFE_INTERNAL_FORWARD_LIMIT =
      45; // 45 degree hood angle (50.4 max)
  public static final double HOOD_INTERNAL_FORWARD_LIMIT = 46; // 46 degree hood angle (50.4 max)

  public static final double HOOD_REVERSE_LIMIT = 5; // was 15...need this for close-to-hub shooting
  public static final double HOOD_INTERNAL_REVERSE_LIMIT = 5; // 15 degree hood angle (2.4 min)
  public static final double HOOD_ENCODER_POSITION_CONVERSION_FACTOR = 360;

  public static final double ACCEL_COMP_FACTOR = 0.100; // in units of seconds

  private static final Point2D[] kHoodPoints =
      new Point2D.Double[] {
        // (distance, hood angle)
        // Old data - what we ran at Durham
        // new Point2D.Double(1.0, 15),
        // new Point2D.Double(1.5, 20),
        // new Point2D.Double(2.25, 20),
        // new Point2D.Double(3.0, 20),
        // new Point2D.Double(3.46, 25),
        // new Point2D.Double(4.0, 25),
        // new Point2D.Double(4.25, 25),
        // new Point2D.Double(4.94, 25),
        // new Point2D.Double(5.36, 25),

        new Point2D.Double(1.15, 5),
        new Point2D.Double(2.5, 20),
        new Point2D.Double(3.15, 25),
        new Point2D.Double(3.5, 27.5),
        new Point2D.Double(4.01, 30),
        new Point2D.Double(5.36, 35),
      };

  public static final LinearInterpolationTable kHoodTable =
      new LinearInterpolationTable(kHoodPoints);

  private static final Point2D[] kHoodGoalPoints =
      new Point2D.Double[] {
        // (distance, hood angle)
        // goal shots
        new Point2D.Double(4.5, 20),
        new Point2D.Double(6.0, 35),
        new Point2D.Double(8, 45),
        new Point2D.Double(10.8, 46),
        new Point2D.Double(12.13, 46),
        new Point2D.Double(14.0, 46),
      };

  public static final LinearInterpolationTable kHoodGoalTable =
      new LinearInterpolationTable(kHoodGoalPoints);

  private static final Point2D[] kHoodAngles =
      new Point2D.Double[] {
        // (Hood Angle, Through Bore Encoder Angle)
        new Point2D.Double(HOOD_FORWARD_LIMIT, HOOD_INTERNAL_FORWARD_LIMIT),
        new Point2D.Double(HOOD_SAFE_FORWARD_LIMIT, HOOD_SAFE_INTERNAL_FORWARD_LIMIT),
        new Point2D.Double(HOOD_REVERSE_LIMIT, HOOD_INTERNAL_REVERSE_LIMIT)
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
