package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.ClimbConstants.NEO2Constants;

public class ClimbSimNEO implements ClimbIO {

  private double positionMm = 0.0;
  private double velocityMmPerSec = 0.0;
  private double appliedVolts = 0.0;

  private double setpoint = 0.0;
  private boolean closedLoop = false;
  private boolean fastMode = true;

  private boolean limitsEnabled = true;

  // Tunable simulation constants
  private static final double MAX_SPEED_MM_PER_SEC = 500.0; // fake max speed
  private static final double LOOP_PERIOD = 0.02; // 20ms

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // --- Closed-loop control ---
    if (closedLoop) {
      double error = setpoint - positionMm;
      double kP = NEO2Constants.CLIMB_KP;

      double output = kP * error;

      // Clamp based on fast/slow mode
      double maxOutput = fastMode ? 1.0 : 0.8;
      output = Math.max(-maxOutput, Math.min(maxOutput, output));

      appliedVolts = output * 12.0;
    }

    // --- Convert volts to velocity ---
    velocityMmPerSec = (appliedVolts / 12.0) * MAX_SPEED_MM_PER_SEC;

    // --- Integrate position ---
    positionMm += velocityMmPerSec * LOOP_PERIOD;

    // --- Apply soft limits ---
    if (limitsEnabled) {
      double max = ClimbConstants.EXTEND_HEIGHT.in(edu.wpi.first.units.Units.Millimeters);

      if (positionMm < 0) {
        positionMm = 0;
        velocityMmPerSec = 0;
      } else if (positionMm > max) {
        positionMm = max;
        velocityMmPerSec = 0;
      }
    }

    // --- Populate inputs ---
    inputs.position = positionMm;
    inputs.appliedVolts = appliedVolts;
    inputs.limitsEnabled = limitsEnabled;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    velocityMmPerSec = 0.0;
    closedLoop = false;
  }

  @Override
  public void setPos(double pose, boolean fast) {
    setpoint = pose;
    fastMode = fast;
    closedLoop = true;
  }

  @Override
  public void dutyCycle(double value) {
    appliedVolts = value * 12.0;
    closedLoop = false;
  }

  @Override
  public boolean getLimitsEnabled() {
    return limitsEnabled;
  }

  @Override
  public void configure(boolean limitsEnabled) {
    this.limitsEnabled = limitsEnabled;

    if (limitsEnabled) {
      positionMm = 0.0;
    }
  }
}
