package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;

public class ClimbConstants {
  public static final double CLIMB_GEAR_RATIO = 81;
  public static final Distance CLIMB_SPOOL_DIAMETER = Millimeters.of(20.7);
  public static final Distance EXTEND_HEIGHT = Millimeters.of(30.0); // FIXME: Figure this out
  public static final Distance CLIMB_HEIGHT = Millimeters.of(20.0); // FIXME: Figure this out

  public static class KrakenConstants {
    // FIXME: These constants are entirely fake.

    public static final double CLIMB_KP = 1.0;
    public static final double CLIMB_KD = 0.0;
    public static final double CLIMB_KS = 0.21;
    public static final double CLIMB_KV = 3.2;
    public static final double CLIMB_KA = 0.0;
    public static final double CLIMB_KI = 0;
    public static final double CLIMB_KG = 0.4;
  }

  public static class NEO2Constants {
    public static final double CLIMB_KP = 0.2;
    public static final double CLIMB_KS = 0.0;
    public static final double CLIMB_KG = 0.0;
  }
}
