package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private Angle hoodAngle = Degrees.of(0);

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
    Logger.recordOutput("Hood/HoodAngle", angle.in(Degrees));
    Logger.recordOutput("Hood/HoodAngleDashboard", angle.in(Degrees) + 0);
    hoodIO.setPositionHood(angle);
  }

  public void stop() {
    hoodIO.stopHood();
  }

  public Angle getPositionHood() {
    return hoodAngle;
  }
}
