package frc.robot.subsystems.augers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Augers extends SubsystemBase { // Defines every Subsystem
  private final AugersIO
      augersIO; // Instance of AugersIO interface that becomes AugersReal or AugersSim
  private final AugersIOInputsAutoLogged augersInputs = new AugersIOInputsAutoLogged(); // Instance

  public Augers(
      AugersIO augersIO) { // Constructor: Mandatory to new class; [NewClass] = python '__init__'
    this.augersIO = augersIO; // Defining a class variable; 'this' = python 'self'
  }

  @Override // Overrides parent definition of periodic()
  public void periodic() { // Runs periodically
    augersIO.updateInputs(augersInputs); // Processes augersInputs for logging
    Logger.processInputs("Augers", augersInputs); // Logs augersInputs under "Augers"
  }

  private void setVoltage(double voltage) { // Sets voltage
    Logger.recordOutput("Augers/SetVoltage", voltage); // Logs voltage under a category in "Augers"
    augersIO.setVoltage(voltage); // References the setVoltage command in augersIO
  }

  public void feed() { // Sets voltage to forward value
    setVoltage(AugersConstants.FEED_VOLTAGE); // References preset value
  }

  public void reverse() { // Sets voltage to reverse value
    setVoltage(AugersConstants.REVERSE_VOLTAGE); // References preset value
  }

  public void stop() { // Sets voltage to 0
    setVoltage(0.0); // Sets voltage to 0
  }
}
