package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkUtil;

public class IntakeReal implements IntakeIO {
  public final SparkBase rollerMotor = new SparkMax(CAN.INTAKE_ROLLERS, MotorType.kBrushless);

  public IntakeReal() {
    var rollerConfig = new SparkMaxConfig();
    rollerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.ROLLER_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    SparkUtil.tryUntilOk(
        rollerMotor,
        5,
        () ->
            rollerMotor.configure(
                rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerAppliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.rollerVelocity = rollerMotor.getEncoder().getVelocity();
    inputs.rollerCurrent = rollerMotor.getOutputCurrent();
    double rollerMotorTemp = rollerMotor.getMotorTemperature();
    REVLibError rollerMotorLastError = rollerMotor.getLastError();
    if (rollerMotorLastError != REVLibError.kOk || rollerMotorTemp == 0) {
      inputs.intakeStatus = false;
    } else {
      inputs.intakeStatus = true;
    }
  }

  public void runIntakeRoller(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void intake(double dutyCycle) {
    rollerMotor.set(dutyCycle);
  }
}
