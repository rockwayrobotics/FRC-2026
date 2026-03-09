package frc.robot.subsystems.kicker;

import frc.robot.util.SimUtils;

public class KickerSim implements KickerIO {
  private double kickerVelocity = 0.0;
  private double kickerSetpoint = 0.0;
  private boolean stopped = false;

  private static final double KICKER_ACCEL_RATE = 1000.0; // RPM per second

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    if (!stopped) {
      kickerVelocity = SimUtils.slew(kickerVelocity, kickerSetpoint, KICKER_ACCEL_RATE);
    } else {
      kickerVelocity = SimUtils.slew(kickerVelocity, 0.0, KICKER_ACCEL_RATE);
    }

    inputs.kickerVelocity = kickerVelocity;
    inputs.kickerStatus = true;
  }

  @Override
  public void setVelocityKicker(double RPM) {
    stopped = false;
    kickerSetpoint = RPM;
  }

  @Override
  public void stop() {
    stopped = true;
  }
}
