package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  private double flywheelRPMSetpoint = 0.0;
  private double flywheelRPM = 0.0;

  private boolean operatorOverride = false;

  private boolean autoControlled = false;
  private double autoSetpointRPM = 0.0;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    if (DriverStation.isAutonomous() && autoControlled) {
      setVelocityFlywheel(autoSetpointRPM);
    }

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

  public void setOperatorOverride(boolean override) {
    Logger.recordOutput("Shooter/OperatorOverride", override);
    operatorOverride = override;
  }

  public boolean isOperatorOverriding() {
    return this.operatorOverride;
  }

  public void setAutoControlled(boolean auto) {
    this.autoControlled = auto;
  }

  public void setAutoSetpointRPM(double setpoint) {
    double rpm = MathUtil.clamp(setpoint, 2000, ShooterConstants.FLYWHEEL_MAX_RPM);
    this.autoSetpointRPM = rpm;
  }
}
