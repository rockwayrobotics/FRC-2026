package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }

  public void setVelocityFlywheel(double RPM) {
    Logger.recordOutput("Shooter/ShooterVelocityRPM", RPM);
    shooterIO.setVelocityFlywheel(RPM);
  }

  public void setVelocityKicker(double RPM) {
    Logger.recordOutput("Shooter/KickerVelocityRPM", RPM);
    shooterIO.setVelocityKicker(RPM);
  }

  public void setPositionHood(Angle angle) {
    Logger.recordOutput("Shooter/HoodAngle", angle.in(Degrees));
    Logger.recordOutput("Shooter/HoodAngleDashboard", angle.in(Degrees) + 0);
    shooterIO.setPositionHood(angle);
  }

  public void stop() {
    setVelocityKicker(0);
    shooterIO.stopHood();
    setVelocityFlywheel(0);
  }
}
