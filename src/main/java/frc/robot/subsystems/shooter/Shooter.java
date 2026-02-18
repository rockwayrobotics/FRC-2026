package frc.robot.subsystems.shooter;

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

  public void setVoltageShooter(double voltage) {
    Logger.recordOutput("Shooter/ShooterVoltage", voltage);
    shooterIO.setVoltageShooter(voltage);
  }

  public void setVoltageKicker(double voltage) {
    Logger.recordOutput("Shooter/KickerVoltage", voltage);
    shooterIO.setVoltageKicker(voltage);
  }

  public void setVoltageHood(double voltage) {
    Logger.recordOutput("Shooter/HoodVoltage", voltage);
    shooterIO.setVoltageHood(voltage);
  }

  public void stop() {
    setVoltageKicker(0);
    setVoltageHood(0);
    setVoltageShooter(0);
  }
}
