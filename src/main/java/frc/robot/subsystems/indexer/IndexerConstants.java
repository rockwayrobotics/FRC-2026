package frc.robot.subsystems.indexer;

public class IndexerConstants {
  public static final double AUGERS_GEAR_RATIO = 12;
  public static final double KICKER_GEAR_RATIO = 1;
  public static final double FEED_VELOCITY_RPM =
      200; // FIXME: we want 60% speed but we don't know what that is...
  public static final double REVERSE_VELOCITY_RPM =
      -200; // FIXME: we want something slow but we don't know
  // Augers PIDV Constants
  public static final double AUGERS_KP = 0.0;
  public static final double AUGERS_KI = 0.0;
  public static final double AUGERS_KD = 0.0;
  public static final double AUGERS_KV = 0.0;
  // Kicker PIDV Constants
  public static final double KICKER_KP = 0.0;
  public static final double KICKER_KI = 0.0;
  public static final double KICKER_KD = 0.0;
  public static final double KICKER_KV = 0.0;
}
