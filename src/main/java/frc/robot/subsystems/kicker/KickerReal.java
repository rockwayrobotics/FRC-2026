package frc.robot.subsystems.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class KickerReal implements KickerIO {
  private final SparkBase kickerMotor = new SparkMax(CAN.KICKER, MotorType.kBrushless);
  private final RelativeEncoder kickerEncoder = kickerMotor.getEncoder();
  private final SparkClosedLoopController kickerController = kickerMotor.getClosedLoopController();

  public KickerReal() {
    var kickerConfig = new SparkMaxConfig();
    kickerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    kickerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(KickerConstants.KICKER_KP, KickerConstants.KICKER_KI, KickerConstants.KICKER_KD);
    kickerConfig.closedLoop.feedForward.kV(KickerConstants.KICKER_KV);

    kickerConfig.encoder.velocityConversionFactor(1.0 / KickerConstants.KICKER_GEAR_RATIO);
    SparkUtil.tryUntilOk(
        kickerMotor,
        5,
        () ->
            kickerMotor.configure(
                kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    SparkUtil.ifOk(
        kickerMotor, kickerEncoder::getVelocity, (value) -> inputs.kickerVelocity = value);
    SparkUtil.ifOk(
        kickerMotor,
        new DoubleSupplier[] {kickerMotor::getAppliedOutput, kickerMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    double kickerTemp = kickerMotor.getMotorTemperature();
    REVLibError kickerMotorLastError = kickerMotor.getLastError();
    if (kickerMotorLastError != REVLibError.kOk || kickerTemp == 0) {
      inputs.kickerStatus = false;
    } else {
      inputs.kickerStatus = true;
    }
  }

  @Override
  public void stop() {
    kickerMotor.set(0);
  }

  @Override
  public void setVelocityKicker(double RPM) {
    kickerController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}
