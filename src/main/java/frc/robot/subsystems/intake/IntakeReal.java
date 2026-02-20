package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class IntakeReal implements IntakeIO {
  public final SparkBase extendRetractMotor =
      new SparkMax(CAN.INTAKE_RETRACTION, MotorType.kBrushless);
  public final SparkBase rollerMotor =
      new SparkMax(CAN.INTAKE_EXTENDING_ROLLERS, MotorType.kBrushless);
  private final RelativeEncoder retractEncoder = extendRetractMotor.getEncoder();
  private final RelativeEncoder extendingEncoder = rollerMotor.getEncoder();

  public IntakeReal() {
    var retractConfig = new SparkMaxConfig();
    retractConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    retractConfig.encoder.positionConversionFactor(IntakeConstants.RETRACT_GEAR_RATIO);
    retractConfig
        .softLimit
        .forwardSoftLimit(IntakeConstants.RETRACT_FORWARD_LIMIT)
        .reverseSoftLimit(IntakeConstants.RETRACT_BACKWARD_LIMIT)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    SparkUtil.tryUntilOk(
        extendRetractMotor,
        5,
        () ->
            extendRetractMotor.configure(
                retractConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var extendingConfig = new SparkMaxConfig();
    extendingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    SparkUtil.tryUntilOk(
        rollerMotor,
        5,
        () ->
            rollerMotor.configure(
                extendingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    SparkUtil.ifOk(
        extendRetractMotor, retractEncoder::getPosition, (value) -> inputs.retractPosition = value);
    SparkUtil.ifOk(
        extendRetractMotor,
        new DoubleSupplier[] {
          extendRetractMotor::getAppliedOutput, extendRetractMotor::getBusVoltage
        },
        (values) -> inputs.retractAppliedVolts = values[0] * values[1]);

    SparkUtil.ifOk(
        rollerMotor,
        extendingEncoder::getVelocity,
        (value) -> inputs.extendingVelocityRadsPerSec = value);
    SparkUtil.ifOk(
        rollerMotor,
        new DoubleSupplier[] {rollerMotor::getAppliedOutput, rollerMotor::getBusVoltage},
        (values) -> inputs.extendingAppliedVolts = values[0] * values[1]);
    inputs.rollerVelocity = rollerMotor.getEncoder().getVelocity();
    inputs.rollerCurrent = rollerMotor.getOutputCurrent();
    double rollerMotorTemp = rollerMotor.getMotorTemperature();
    REVLibError rollerMotorLastError = rollerMotor.getLastError();
    double extendRetractMotorTemp = extendRetractMotor.getMotorTemperature();
    REVLibError extendRetractMotorLastError = extendRetractMotor.getLastError();
    if (rollerMotorLastError != REVLibError.kOk
        || extendRetractMotorLastError != REVLibError.kOk
        || rollerMotorTemp == 0
        || extendRetractMotorTemp == 0) {
      inputs.intakeStatus = false;
    } else {
      inputs.intakeStatus = true;
    }
  }

  public void runIntakeRoller(double volts) {
    rollerMotor.setVoltage(volts);
  }

  public void intakeInOut(double limit) {
    extendRetractMotor
        .getClosedLoopController()
        .setSetpoint(limit, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
