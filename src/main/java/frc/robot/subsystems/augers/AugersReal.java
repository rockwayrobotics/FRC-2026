package frc.robot.subsystems.augers;

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

public class AugersReal implements AugersIO {
  private final SparkBase motor = new SparkMax(CAN.AUGER, MotorType.kBrushless);
  private final RelativeEncoder motorEncoder = motor.getEncoder();

  public AugersReal() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    // FIXME: Do we care about encoder velocity conversion?
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(AugersIOInputs inputs) {
    SparkUtil.ifOk(motor, motorEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    SparkUtil.ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
