package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class IntakeReal implements IntakeIO {
  private final SparkBase retract = new SparkMax(CAN.INTAKE_RETRACTION, MotorType.kBrushless);
  private final SparkBase extendingRoller =
      new SparkMax(CAN.INTAKE_EXTENDING_ROLLERS, MotorType.kBrushless);
  private final RelativeEncoder retractEncoder = retract.getEncoder();
  private final RelativeEncoder extendingEncoder = extendingRoller.getEncoder();

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
        retract,
        5,
        () ->
            retract.configure(
                retractConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var extendingConfig = new SparkMaxConfig();
    extendingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    SparkUtil.tryUntilOk(
        extendingRoller,
        5,
        () ->
            extendingRoller.configure(
                extendingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    SparkUtil.ifOk(retract, retractEncoder::getPosition, (value) -> inputs.retractPosition = value);
    SparkUtil.ifOk(
        retract,
        new DoubleSupplier[] {retract::getAppliedOutput, retract::getBusVoltage},
        (values) -> inputs.retractAppliedVolts = values[0] * values[1]);

    SparkUtil.ifOk(
        extendingRoller,
        extendingEncoder::getVelocity,
        (value) -> inputs.extendingVelocityRadsPerSec = value);
    SparkUtil.ifOk(
        extendingRoller,
        new DoubleSupplier[] {extendingRoller::getAppliedOutput, extendingRoller::getBusVoltage},
        (values) -> inputs.extendingAppliedVolts = values[0] * values[1]);
  }
}
