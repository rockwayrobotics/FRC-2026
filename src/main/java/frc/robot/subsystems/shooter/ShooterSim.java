package frc.robot.subsystems.shooter;

import frc.robot.util.SimUtils;

public class ShooterSim implements ShooterIO {
  private double flywheelVelocity = 0.0;
  private double flywheelAppliedVolts = 0.0;
  private double flywheelSetpoint = 0.0;

  // Simulation tuning constants
  private static final double FLYWHEEL_ACCEL_RATE = 1000.0; // RPM per second
  private static final double FLYWHEEL_DECEL_RATE = 300.0; // RPM per second (natural decay)

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Simulate flywheel velocity ramping toward setpoint
    double accelRate =
        flywheelSetpoint > flywheelVelocity ? FLYWHEEL_ACCEL_RATE : FLYWHEEL_DECEL_RATE;
    flywheelVelocity = SimUtils.slew(flywheelVelocity, flywheelSetpoint, accelRate);

    // Simulate applied volts proportional to velocity (rough approximation)
    flywheelAppliedVolts = (flywheelVelocity / 6000.0) * 12.0;

    inputs.flywheelVelocity = flywheelVelocity;
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.shooterStatus = true;
  }

  @Override
  public void setVelocityFlywheel(double RPM) {
    flywheelSetpoint = RPM;
  }

  @Override
  public void stopFlywheel() {
    flywheelSetpoint = 0.0;
  }
}
