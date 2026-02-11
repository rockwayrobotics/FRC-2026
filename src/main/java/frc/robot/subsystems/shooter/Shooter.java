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
}
