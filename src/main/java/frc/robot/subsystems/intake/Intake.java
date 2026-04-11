package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  private final LoggedNetworkNumber rollerTest =
      new LoggedNetworkNumber("Intake/RollerTestDuty", -0.4);
  private final LoggedNetworkBoolean rollerOvercurrent =
      new LoggedNetworkBoolean("Intake/RollerOverCurrent", false);
  private boolean rollerOverCurrentAuto = false;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    rollerOvercurrent.set(intakeInputs.rollerCurrent > IntakeConstants.ROLLER_CURRENT_LIMIT - 0.5);
    rollerOverCurrentAuto = intakeInputs.rollerCurrent > 50;
    Logger.processInputs("Intake", intakeInputs);
  }

  public boolean getOverCurrent() {
    return rollerOvercurrent.get();
  }

  public boolean getOverCurrentAuto() {
    return rollerOverCurrentAuto;
  }

  public void intake(double dutyCycle) {
    intakeIO.intake(dutyCycle);
  }

  public void intakeTest() {
    double dutyCycle = MathUtil.clamp(rollerTest.getAsDouble(), -1, 1);
    intakeIO.intake(dutyCycle);
  }
}
