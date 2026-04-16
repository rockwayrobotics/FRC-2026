package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ShooterReal implements ShooterIO {
  private final SparkBase flywheelLeader = new SparkFlex(CAN.FLYWHEEL_LEADER, MotorType.kBrushless);
  //   private final SparkBase flywheelFollower1 =
  //       new SparkFlex(CAN.FLYWHEEL_FOLLOWER_1, MotorType.kBrushless);
  private final SparkBase flywheelFollower2 =
      new SparkFlex(CAN.FLYWHEEL_FOLLOWER_2, MotorType.kBrushless);

  private final RelativeEncoder flywheelEncoder = flywheelLeader.getEncoder();
  private final SparkClosedLoopController flywheelLeaderController =
      flywheelLeader.getClosedLoopController();

  public ShooterReal() {
    var flywheelLeaderConfig = new SparkFlexConfig();
    flywheelLeaderConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(80)
        .voltageCompensation(12.0);
    flywheelLeaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            (ShooterConstants.FLYWHEEL_KP),
            (ShooterConstants.FLYWHEEL_KI),
            (ShooterConstants.FLYWHEEL_KD));
    flywheelLeaderConfig.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_KV);
    flywheelLeaderConfig.encoder.velocityConversionFactor(
        1.0 / ShooterConstants.FLYWHEEL_GEAR_RATIO);
    SparkUtil.tryUntilOk(
        flywheelLeader,
        5,
        () ->
            flywheelLeader.configure(
                flywheelLeaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var flywheelFollowerConfig = new SparkFlexConfig();
    flywheelFollowerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(80)
        .voltageCompensation(12.0)
        .follow(CAN.FLYWHEEL_LEADER, true);
    // SparkUtil.tryUntilOk(
    //     flywheelFollower1,
    //     5,
    //     () ->
    //         flywheelFollower1.configure(
    //             flywheelFollowerConfig,
    //             ResetMode.kResetSafeParameters,
    //             PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        flywheelFollower2,
        5,
        () ->
            flywheelFollower2.configure(
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    SparkUtil.ifOk(
        flywheelLeader, flywheelEncoder::getVelocity, (value) -> inputs.flywheelVelocity = value);
    SparkUtil.ifOk(
        flywheelLeader,
        new DoubleSupplier[] {flywheelLeader::getAppliedOutput, flywheelLeader::getBusVoltage},
        (values) -> inputs.flywheelAppliedVolts = values[0] * values[1]);
    // FIXME: Should we log applied for followers?

    SparkUtil.ifOk(
        flywheelLeader,
        flywheelLeader::getOutputCurrent,
        (value) -> inputs.flywheelLeaderCurrent = value);
    SparkUtil.ifOk(
        flywheelFollower2,
        flywheelFollower2::getOutputCurrent,
        (value) -> inputs.flywheelFollower2Current = value);
    SparkUtil.ifOk(
        flywheelLeader,
        flywheelLeader::getAppliedOutput,
        (value) -> inputs.flywheelLeaderAppliedOutput = value);
    SparkUtil.ifOk(
        flywheelFollower2,
        flywheelFollower2::getAppliedOutput,
        (value) -> inputs.flywheelFollower2AppliedOutput = value);

    double flywheelLeaderTemp = flywheelLeader.getMotorTemperature();
    REVLibError flywheelLeaderLastError = flywheelLeader.getLastError();
    // double flywheelFollower1Temp = flywheelFollower1.getMotorTemperature();
    // REVLibError flywheelFollower1LastError = flywheelFollower1.getLastError();
    double flywheelFollower2Temp = flywheelFollower2.getMotorTemperature();
    REVLibError flywheelFollower2LastError = flywheelFollower2.getLastError();
    if (flywheelLeaderLastError != REVLibError.kOk
        || flywheelLeaderTemp == 0
        // || flywheelFollower1LastError != REVLibError.kOk
        // || flywheelFollower1Temp == 0
        || flywheelFollower2LastError != REVLibError.kOk
        || flywheelFollower2Temp == 0) {
      inputs.shooterStatus = false;
    } else {
      inputs.shooterStatus = true;
    }
  }

  @Override
  public void configureLeader(double kp, double ki, double kd, double kv) {
    var flywheelLeaderConfig = new SparkFlexConfig();
    flywheelLeaderConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(60)
        .voltageCompensation(12.0);
    flywheelLeaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kp, ki, kd);
    //     (ShooterConstants.FLYWHEEL_KP),
    //     (ShooterConstants.FLYWHEEL_KI),
    //     (ShooterConstants.FLYWHEEL_KD));
    flywheelLeaderConfig.closedLoop.feedForward.kV(kv); // ShooterConstants.FLYWHEEL_KV);
    flywheelLeaderConfig.encoder.velocityConversionFactor(
        1.0 / ShooterConstants.FLYWHEEL_GEAR_RATIO);
    SparkUtil.tryUntilOk(
        flywheelLeader,
        5,
        () ->
            flywheelLeader.configure(
                flywheelLeaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void setVelocityFlywheel(double RPM) {
    flywheelLeaderController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stopFlywheel() {
    flywheelLeader.stopMotor();
  }
}
