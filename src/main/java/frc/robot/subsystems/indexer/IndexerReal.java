package frc.robot.subsystems.indexer;

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

public class IndexerReal implements IndexerIO {
  private final SparkBase augersMotor = new SparkMax(CAN.AUGER, MotorType.kBrushless);
  private final SparkBase kickerMotor = new SparkMax(CAN.KICKER, MotorType.kBrushless);
  private final RelativeEncoder augersEncoder = augersMotor.getEncoder();
  private final RelativeEncoder kickerEncoder = kickerMotor.getEncoder();
  private final SparkClosedLoopController augersController = augersMotor.getClosedLoopController();
  private final SparkClosedLoopController kickerController = kickerMotor.getClosedLoopController();

  public IndexerReal() {
    var augersConfig = new SparkMaxConfig();
    augersConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    augersConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IndexerConstants.AUGERS_KP, IndexerConstants.AUGERS_KI, IndexerConstants.AUGERS_KD);
    augersConfig.closedLoop.feedForward.kV(IndexerConstants.AUGERS_KV);
    augersConfig.encoder.velocityConversionFactor(1.0 / IndexerConstants.AUGERS_GEAR_RATIO);
    SparkUtil.tryUntilOk(
        augersMotor,
        5,
        () ->
            augersMotor.configure(
                augersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var kickerConfig = new SparkMaxConfig();
    kickerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    kickerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IndexerConstants.KICKER_KP, IndexerConstants.KICKER_KI, IndexerConstants.KICKER_KD);
    kickerConfig.closedLoop.feedForward.kV(IndexerConstants.KICKER_KV);

    kickerConfig.encoder.velocityConversionFactor(1.0 / IndexerConstants.KICKER_GEAR_RATIO);
    SparkUtil.tryUntilOk(
        kickerMotor,
        5,
        () ->
            kickerMotor.configure(
                kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    SparkUtil.ifOk(
        augersMotor, augersEncoder::getVelocity, (value) -> inputs.augerVelocityRPM = value);
    SparkUtil.ifOk(
        augersMotor,
        new DoubleSupplier[] {augersMotor::getAppliedOutput, augersMotor::getBusVoltage},
        (values) -> inputs.augerAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        kickerMotor, kickerEncoder::getVelocity, (value) -> inputs.kickerVelocity = value);
    double augerMotorTemp = augersMotor.getMotorTemperature();
    REVLibError augerMotorLastError = augersMotor.getLastError();
    double kickerTemp = kickerMotor.getMotorTemperature();
    REVLibError kickerMotorLastError = kickerMotor.getLastError();
    if (augerMotorLastError != REVLibError.kOk
        || augerMotorTemp == 0
        || kickerMotorLastError != REVLibError.kOk
        || kickerTemp == 0) {
      inputs.indexerStatus = false;
    } else {
      inputs.indexerStatus = true;
    }
  }

  @Override
  public void stop() {
    augersMotor.set(0);
    kickerMotor.set(0);
  }

  @Override
  public void setVelocityAugers(double RPM) {
    augersController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVelocityKicker(double RPM) {
    kickerController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}
