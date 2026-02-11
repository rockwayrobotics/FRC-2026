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
}
