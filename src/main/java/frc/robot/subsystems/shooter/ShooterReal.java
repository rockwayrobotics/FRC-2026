package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ShooterReal implements ShooterIO {
  private final SparkBase flywheelLeader = new SparkFlex(CAN.FLYWHEEL_LEADER, MotorType.kBrushless);
  private final SparkBase flywheelFollower1 =
      new SparkFlex(CAN.FLYWHEEL_FOLLOWER_1, MotorType.kBrushless);
  private final SparkBase flywheelFollower2 =
      new SparkFlex(CAN.FLYWHEEL_FOLLOWER_2, MotorType.kBrushless);

  private final SparkBase hood = new SparkMax(CAN.VARIABLE_HOOD, MotorType.kBrushless);
  private final SparkBase kicker = new SparkMax(CAN.KICKER, MotorType.kBrushless);

  private final RelativeEncoder flywheelEncoder = flywheelLeader.getEncoder();
  private final AbsoluteEncoder hoodEncoder = hood.getAbsoluteEncoder();
  private final RelativeEncoder kickerEncoder = kicker.getEncoder();

  public ShooterReal() {
    var flywheelLeaderConfig = new SparkFlexConfig();
    flywheelLeaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60).voltageCompensation(12.0);
    // FIXME: Add PID constants here. Tunable?
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
        .smartCurrentLimit(60)
        .voltageCompensation(12.0)
        .follow(CAN.FLYWHEEL_LEADER, true);
    SparkUtil.tryUntilOk(
        flywheelFollower1,
        5,
        () ->
            flywheelFollower1.configure(
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        flywheelFollower2,
        5,
        () ->
            flywheelFollower2.configure(
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    var hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    hoodConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    hoodConfig
        .softLimit
        .forwardSoftLimit(ShooterConstants.HOOD_FORWARD_LIMIT)
        .reverseSoftLimit(ShooterConstants.HOOD_REVERSE_LIMIT)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    SparkUtil.tryUntilOk(
        hood,
        5,
        () ->
            hood.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var kickerConfig = new SparkMaxConfig();
    kickerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    // FIXME: Could add velocity conversion, do we need PID?
    SparkUtil.tryUntilOk(
        kicker,
        5,
        () ->
            kicker.configure(
                kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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

    SparkUtil.ifOk(hood, hoodEncoder::getPosition, (value) -> inputs.hoodPosition = value);

    SparkUtil.ifOk(kicker, kickerEncoder::getVelocity, (value) -> inputs.kickerVelocity = value);
    double flywheelLeaderTemp = flywheelLeader.getMotorTemperature();
    REVLibError flywheelLeaderLastError = flywheelLeader.getLastError();
    double flywheelFollower1Temp = flywheelFollower1.getMotorTemperature();
    REVLibError flywheelFollower1LastError = flywheelFollower1.getLastError();
    double flywheelFollower2Temp = flywheelFollower2.getMotorTemperature();
    REVLibError flywheelFollower2LastError = flywheelFollower2.getLastError();
    double hoodTemp = hood.getMotorTemperature();
    REVLibError hoodLastError = hood.getLastError();
    double kickerTemp = kicker.getMotorTemperature();
    REVLibError kickerLastError = kicker.getLastError();
    if (flywheelLeaderLastError != REVLibError.kOk
        || flywheelLeaderTemp == 0
        || flywheelFollower1LastError != REVLibError.kOk
        || flywheelFollower1Temp == 0
        || flywheelFollower2LastError != REVLibError.kOk
        || flywheelFollower2Temp == 0
        || hoodLastError != REVLibError.kOk
        || hoodTemp == 0
        || kickerLastError != REVLibError.kOk
        || kickerTemp == 0) {
      inputs.shooterStatus = false;
    } else {
      inputs.shooterStatus = true;
    }
  }
}
