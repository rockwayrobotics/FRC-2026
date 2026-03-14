package frc.robot.subsystems.intakeExtender;

public class IntakeExtenderConstants {
  public static final double RETRACT_GEAR_RATIO = 5 * 9 * 60 / 48.0;

  // Limits in degrees (was from 12 to -110)
  public static final double EXTEND_LIMIT = 122.0;
  public static final double RETRACT_LIMIT = 0.0;

  // 20% Duty cycle on the retract seems to make it work perfectly (rotates and slides in)
  // 10% duty cycle on the extend works as well.
  // FIXME: Does this work if there are balls?
  public static final double EXTEND_DUTY_CYCLE = 0.6;
  public static final double RETRACT_DUTY_CYCLE = -0.8;

  // FIXME: Unclear if this is reasonable
  public static final double CURRENT_LIMIT = 10.0;
}
