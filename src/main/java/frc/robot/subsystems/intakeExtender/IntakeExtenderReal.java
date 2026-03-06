package frc.robot.subsystems.intakeExtender;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
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

public class IntakeExtenderReal implements IntakeExtenderIO {
  public final SparkBase extendRetractMotor =
      new SparkMax(CAN.INTAKE_RETRACTION, MotorType.kBrushless);
  private final RelativeEncoder extendRetractEncoder = extendRetractMotor.getEncoder();

  public IntakeExtenderReal() {
    var extendRetractConfig = new SparkMaxConfig();
    extendRetractConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(20)
        .voltageCompensation(12.0);
    // Convert from revolutions to degrees, accounting for the gear ratio.
    extendRetractConfig.encoder.positionConversionFactor(
        360.0 / IntakeExtenderConstants.RETRACT_GEAR_RATIO);
    extendRetractConfig
        .softLimit
        .forwardSoftLimit(IntakeExtenderConstants.EXTEND_LIMIT)
        .reverseSoftLimit(IntakeExtenderConstants.RETRACT_LIMIT)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    SparkUtil.tryUntilOk(
        extendRetractMotor,
        5,
        () ->
            extendRetractMotor.configure(
                extendRetractConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    // Set the encoder position to 0 on startup so that the soft limits work correctly.
    extendRetractEncoder.setPosition(0);
    // FIXME: What do we do if we lose power during a match? This will set the position to 0 at
    // startup.
  }

  @Override
  public void updateInputs(IntakeExtenderIOInputs inputs) {
    SparkUtil.ifOk(
        extendRetractMotor,
        extendRetractEncoder::getPosition,
        (value) -> inputs.extendPosition = value);
    SparkUtil.ifOk(
        extendRetractMotor,
        new DoubleSupplier[] {
          extendRetractMotor::getAppliedOutput, extendRetractMotor::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        extendRetractMotor,
        extendRetractMotor::getOutputCurrent,
        (value) -> inputs.outputCurrent = value);

    double extendRetractMotorTemp = extendRetractMotor.getMotorTemperature();
    REVLibError extendRetractMotorLastError = extendRetractMotor.getLastError();
    if (extendRetractMotorLastError != REVLibError.kOk || extendRetractMotorTemp == 0) {
      inputs.intakeExtenderStatus = false;
    } else {
      inputs.intakeExtenderStatus = true;
    }
  }

  @Override
  public void extend(double dutyCycle) {
    extendRetractMotor.set(dutyCycle);
  }
}
