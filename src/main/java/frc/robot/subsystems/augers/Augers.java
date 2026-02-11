package frc.robot.subsystems.augers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Augers extends SubsystemBase {
  private final AugersIO augersIO;
  private final AugersIOInputsAutoLogged augersInputs = new AugersIOInputsAutoLogged();

  public Augers(AugersIO augersIO) {
    this.augersIO = augersIO;
  }

  @Override
  public void periodic() {
    augersIO.updateInputs(augersInputs);
    Logger.processInputs("Augers", augersInputs);
  }

  private void setVoltage(double voltage) {
    Logger.recordOutput("Augers/SetVoltage", voltage);
    augersIO.setVoltage(voltage);
  }

  public void feed() {
    setVoltage(AugersConstants.FEED_VOLTAGE);
  }

  public void reverse() {
    setVoltage(AugersConstants.REVERSE_VOLTAGE);
  }

  public void stop() {
    setVoltage(0.0);
  }
}
