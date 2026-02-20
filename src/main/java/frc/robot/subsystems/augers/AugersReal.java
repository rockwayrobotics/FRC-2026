package frc.robot.subsystems.augers;

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

public class AugersReal implements AugersIO {
  private final SparkBase augerMotor = new SparkMax(CAN.AUGER, MotorType.kBrushless);
  private final RelativeEncoder motorEncoder = augerMotor.getEncoder();

  public AugersReal() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    // FIXME: Do we care about encoder velocity conversion?
    SparkUtil.tryUntilOk(
        augerMotor,
        5,
        () ->
            augerMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(AugersIOInputs inputs) {
    SparkUtil.ifOk(augerMotor, motorEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    SparkUtil.ifOk(
        augerMotor,
        new DoubleSupplier[] {augerMotor::getAppliedOutput, augerMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    double augerMotorTemp = augerMotor.getMotorTemperature();
    REVLibError augerMotorLastError = augerMotor.getLastError();
    if (augerMotorLastError != REVLibError.kOk || augerMotorTemp == 0) {
      inputs.augerStatus = false;
    } else {
      inputs.augerStatus = true;
    }
  }

  @Override
  public void setVoltage(double volts) {
    augerMotor.setVoltage(volts);
  }
}
