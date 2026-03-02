package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.SimUtils;

public class ShooterSim implements ShooterIO {
  private double flywheelVelocity = 0.0;
  private double flywheelAppliedVolts = 0.0;
  private double hoodRawPosition = ShooterConstants.kHoodAnglesTable.getOutput(15);
  private double flywheelSetpoint = 0.0;
  private double hoodSetpoint = 0.0;
  private boolean hoodStopped = true;

  // Simulation tuning constants
  private static final double FLYWHEEL_ACCEL_RATE = 1000.0; // RPM per second
  private static final double FLYWHEEL_DECEL_RATE = 300.0; // RPM per second (natural decay)
  private static final double HOOD_SLEW_RATE = 10.0; // degrees per second

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Simulate flywheel velocity ramping toward setpoint
    double accelRate =
        flywheelSetpoint > flywheelVelocity ? FLYWHEEL_ACCEL_RATE : FLYWHEEL_DECEL_RATE;
    flywheelVelocity = SimUtils.slew(flywheelVelocity, flywheelSetpoint, accelRate);

    // Simulate applied volts proportional to velocity (rough approximation)
    flywheelAppliedVolts = (flywheelVelocity / 6000.0) * 12.0;

    if (!hoodStopped) {
      // Simulate hood slewing toward setpoint
      hoodRawPosition = SimUtils.slew(hoodRawPosition, hoodSetpoint, HOOD_SLEW_RATE);
    }

    inputs.flywheelVelocity = flywheelVelocity;
    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.hoodRawPosition = hoodRawPosition;
    inputs.hoodPosition = ShooterConstants.kHoodAnglesTable.inverseGet(hoodRawPosition);
    inputs.shooterStatus = true;
  }

  @Override
  public void setVelocityFlywheel(double RPM) {
    flywheelSetpoint = RPM;
  }

  @Override
  public void setPositionHood(Angle angle) {
    hoodStopped = false;
    hoodSetpoint = ShooterConstants.kHoodAnglesTable.getOutput(angle.in(Degrees));
  }

  @Override
  public void stopHood() {
    hoodStopped = true;
  }

  @Override
  public void stopFlywheel() {
    flywheelSetpoint = 0.0;
  }
}
