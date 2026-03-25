package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private Angle hoodAngle = Degrees.of(15);
  private Angle hoodAngleSetpoint = hoodAngle;
  private Angle deferredSetpoint = Degrees.of(15);
  private boolean operatorOverride = false;

  public Hood(HoodIO hoodIO) {
    this.hoodIO = hoodIO;
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    hoodAngle = Degrees.of(hoodInputs.hoodPosition);
    Logger.processInputs("Hood", hoodInputs);
  }

  public void setPositionHood(Angle angle) {
    double degrees = angle.in(Degrees);
    Logger.recordOutput("Hood/RequestedAngle", degrees);
    if (degrees < HoodConstants.HOOD_REVERSE_LIMIT || degrees > HoodConstants.HOOD_FORWARD_LIMIT) {
      return;
    }
    hoodAngleSetpoint = angle;
    Logger.recordOutput("Hood/HoodAngle", angle.in(Degrees));
    Logger.recordOutput("Hood/HoodAngleDashboard", angle.in(Degrees) + 0);
    hoodIO.setPositionHood(angle);
  }

  public void stop() {
    hoodAngleSetpoint = Degrees.of(HoodConstants.HOOD_REST_POINT);
    hoodIO.stopHood();
  }

  public Angle getPositionHood() {
    return hoodAngle;
  }

  public boolean atSetpoint(double degrees) {
    return Math.abs(getPositionHood().minus(getHoodSetpoint()).in(Degrees)) < degrees;
  }

  public Angle getHoodSetpoint() {
    return hoodAngleSetpoint;
  }

  public void setDeferredSetpoint(Angle angle) {
    deferredSetpoint = angle;
  }

  public Angle getDeferredSetpoint() {
    return deferredSetpoint;
  }

  public void setOperatorOverride(boolean override) {
    Logger.recordOutput("Hood/OperatorOverride", override);
    operatorOverride = override;
  }

  public boolean isOperatorOverriding() {
    return this.operatorOverride;
  }
}
