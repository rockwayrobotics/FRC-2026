package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase { // Defines every Subsystem
  private final KickerIO
      kickerIO; // Instance of KickerIO interface that becomes KickerReal or KickerSim
  private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged(); // Instance

  private double rpmSetpoint = 0.0;

  public Kicker(
      KickerIO kickerIO) { // Constructor: Mandatory to new class; [NewClass] = python '__init__'
    this.kickerIO = kickerIO; // Defining a class variable; 'this' = python 'self'
  }

  @Override // Overrides parent definition of periodic()
  public void periodic() { // Runs periodically
    kickerIO.updateInputs(kickerInputs); // Processes kickerInputs for logging
    Logger.processInputs("Kicker", kickerInputs); // Logs kickerInputs under "Kicker"
  }

  public void setVelocityKicker(double RPM) {
    Logger.recordOutput("Kicker/KickerVelocityRPM", RPM);
    kickerIO.setVelocityKicker(RPM);
    this.rpmSetpoint = RPM;
  }

  public double getRpmSetpoint() {
    return rpmSetpoint;
  }

  public void stop() { // Stops all kicker motors.
    kickerIO.stop();
  }
}
