package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO climbIO) {
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbInputs);
    Logger.processInputs("Climb", climbInputs);
  }

  private void setPose(double pose) {
    Logger.recordOutput("Climb/Setpoint", pose);
    climbIO.setPose(pose);
  }
}
