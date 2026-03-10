package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private double flywheelRPMSetpoint = 0.0;
  private double flywheelRPM = 0.0;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    flywheelRPM = shooterInputs.flywheelVelocity;
    Logger.processInputs("Shooter", shooterInputs);
  }

  public void setVelocityFlywheel(double RPM) {
    Logger.recordOutput("Shooter/ShooterVelocityRPM", RPM);
    shooterIO.setVelocityFlywheel(RPM);
    flywheelRPMSetpoint = RPM;
  }

  public void stop() {
    shooterIO.stopFlywheel();
  }

  public double getFlywheelRPMSetpoint() {
    return flywheelRPMSetpoint;
  }

  public double getFlywheelRPM() {
    return flywheelRPM;
  }

  public boolean atFlywheelSetpoint(double toleranceRPM) {
    return Math.abs(flywheelRPM - flywheelRPMSetpoint) < toleranceRPM;
  }

  public void configureLeader(double kp, double ki, double kd, double kv) {
    // shooterIO.configureLeader(kp, ki, kd, kv);
  }
}
