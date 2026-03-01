package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase { // Defines every Subsystem
  private final IndexerIO
      indexerIO; // Instance of IndexerIO interface that becomes IndexerReal or IndexerSim
  private final IndexerIOInputsAutoLogged indexerInputs =
      new IndexerIOInputsAutoLogged(); // Instance

  public Indexer(
      IndexerIO indexerIO) { // Constructor: Mandatory to new class; [NewClass] = python '__init__'
    this.indexerIO = indexerIO; // Defining a class variable; 'this' = python 'self'
  }

  @Override // Overrides parent definition of periodic()
  public void periodic() { // Runs periodically
    indexerIO.updateInputs(indexerInputs); // Processes indexerInputs for logging
    Logger.processInputs("Indexer", indexerInputs); // Logs indexerInputs under "Indexer"
  }

  public void setVelocityAugers(double RPM) { // Sets voltage
    Logger.recordOutput(
        "Indexer/AugersVelocityRPM", RPM); // Logs voltage under a category in "Indexer"
    indexerIO.setVelocityAugers(RPM); // References the setVoltage command in indexerIO
  }

  public void augersFeed() { // Sets voltage to forward value
    setVelocityAugers(IndexerConstants.FEED_VELOCITY_RPM); // References preset value
  }

  public void augersReverse() { // Sets voltage to reverse value
    setVelocityAugers(IndexerConstants.REVERSE_VELOCITY_RPM); // References preset value
  }

  public void setVelocityKicker(double RPM) {
    Logger.recordOutput("Indexer/KickerVelocityRPM", RPM);
    indexerIO.setVelocityKicker(RPM);
  }

  public void stop() { // Stops all indexer motors.
    indexerIO.stop();
  }
}
