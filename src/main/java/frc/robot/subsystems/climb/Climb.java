package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbInputs = new ClimbIOInputsAutoLogged();
  private double setpoint = 0.0;
  private double position = 0.0;

  public Climb(ClimbIO climbIO) {
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbInputs);
    Logger.processInputs("Climb", climbInputs);
    position = climbInputs.position;
  }

  public void stop() {
    climbIO.stop();
  }

  public void setPos(double pose, boolean fast) {
    Logger.recordOutput("Climb/Setpoint", pose);
    climbIO.setPos(pose, fast);
    setpoint = pose;
  }

  public boolean atSetpoint(double toleranceMillimeters) {
    return Math.abs(setpoint - position) < toleranceMillimeters;
  }

  public void extend() {
    setPos(ClimbConstants.EXTEND_HEIGHT.in(Millimeters), true);
  }

  public void retract() {
    setPos(0, true);
  }

  public void climb() {
    setPos(ClimbConstants.CLIMB_HEIGHT.in(Millimeters), true);
  }

  public void unclimb() {
    setPos(ClimbConstants.EXTEND_HEIGHT.in(Millimeters), false);
  }

  public void dutyCycle(double value) {
    climbIO.dutyCycle(value);
  }

  public void toggleLimits() {
    if (climbIO.getLimitsEnabled()) {
      climbIO.configure(false);
    } else {
      climbIO.configure(true);
    }
  }
}
