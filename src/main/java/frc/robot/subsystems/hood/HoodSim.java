package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.SimUtils;

public class HoodSim implements HoodIO {
  private double hoodRawPosition = HoodConstants.kHoodAnglesTable.getOutput(15);
  private double hoodSetpoint = 0.0;

  // Simulation tuning constants
  private static final double HOOD_SLEW_RATE = 10.0; // degrees per second

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Simulate hood slewing toward setpoint
    hoodRawPosition = SimUtils.slew(hoodRawPosition, hoodSetpoint, HOOD_SLEW_RATE);

    inputs.hoodRawPosition = hoodRawPosition;
    inputs.hoodPosition = HoodConstants.kHoodAnglesTable.inverseGet(hoodRawPosition);
    inputs.hoodStatus = true;
  }

  @Override
  public void setPositionHood(Angle angle) {
    hoodSetpoint = HoodConstants.kHoodAnglesTable.getOutput(angle.in(Degrees));
  }

  @Override
  public void stopHood() {
    hoodSetpoint = HoodConstants.kHoodAnglesTable.getOutput(15);
  }
}
