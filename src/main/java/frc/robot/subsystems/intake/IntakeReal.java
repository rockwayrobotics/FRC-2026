package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.CAN;

public class IntakeReal implements IntakeIO {
  private final TalonFX rollerMotor = new TalonFX(CAN.INTAKE_EXTENDING_ROLLERS);
  private final VoltageOut m_rollerRequest = new VoltageOut(0);


  public IntakeReal() {
  MotorOutputConfigs currentConfigs = new MotorOutputConfigs();
  currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
  rollerMotor.getConfigurator().apply(currentConfigs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  public void runIntakeRoller(double volts) {
    rollerMotor.setControl(m_rollerRequest.withOutput(volts));
  }

  public void intakeInOut(double limit) {}
}
