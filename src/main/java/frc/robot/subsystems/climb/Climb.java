package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Millimeters;

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

  public void stop() {
    climbIO.stop();
  }

  private void setPose(double pose, boolean fast) {
    Logger.recordOutput("Climb/Setpoint", pose);
    climbIO.setPose(pose, fast);
  }

  public void extend() {
    setPose(ClimbConstants.EXTEND_HEIGHT.in(Millimeters), true);
  }

  public void retract() {
    setPose(0, true);
  }

  public void climb() {
    setPose(ClimbConstants.CLIMB_HEIGHT.in(Millimeters), false);
  }

  public void unclimb() {
    setPose(ClimbConstants.EXTEND_HEIGHT.in(Millimeters), false);
  }
}
