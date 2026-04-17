package frc.robot.subsystems.intakeExtender;

public class IntakeExtenderConstants {
  // 45:1 gearbox * 4/3 gears
  public static final double RETRACT_GEAR_RATIO = 60;

  // Limits in degrees (was from 12 to -110)
  public static final double EXTEND_LIMIT = 45.5;
  public static final double RETRACT_LIMIT = 5.0;
  public static final double FULL_RETRACT_LIMIT = 0.0;
  public static final double TRASH_COMPACT_EXTEND_LIMIT = 20.0;
  public static final double TRASH_COMPACT_RETRACT_LIMIT = 11.5;

  // 20% Duty cycle on the retract seems to make it work perfectly (rotates and slides in)
  // 10% duty cycle on the extend works as well.
  // FIXME: Does this work if there are balls?
  public static final double EXTEND_DUTY_CYCLE = 0.7;
  public static final double RETRACT_DUTY_CYCLE = -0.7;

  // FIXME: Unclear if this is reasonable
  public static final int CURRENT_LIMIT = 10;
}
