package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
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

  private final RelativeEncoder flywheelEncoder = flywheelLeader.getEncoder();
  private final AbsoluteEncoder hoodEncoder = hood.getAbsoluteEncoder();
  private final SparkClosedLoopController flywheelLeaderController =
      flywheelLeader.getClosedLoopController();
  private final SparkClosedLoopController hoodController = hood.getClosedLoopController();

  public ShooterReal() {
    var flywheelLeaderConfig = new SparkFlexConfig();
    flywheelLeaderConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60).voltageCompensation(12.0);
    flywheelLeaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            (ShooterConstants.FLYWHEEL_KP),
            (ShooterConstants.FLYWHEEL_KI),
            (ShooterConstants.FLYWHEEL_KD));
    flywheelLeaderConfig.closedLoop.feedForward.kV(ShooterConstants.FLYWHEEL_KV);

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
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ShooterConstants.HOOD_KP, ShooterConstants.HOOD_KI, ShooterConstants.HOOD_KD);
    hoodConfig.absoluteEncoder.positionConversionFactor(
        ShooterConstants.HOOD_ENCODER_POSITION_CONVERSION_FACTOR);
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

    double flywheelLeaderTemp = flywheelLeader.getMotorTemperature();
    REVLibError flywheelLeaderLastError = flywheelLeader.getLastError();
    double flywheelFollower1Temp = flywheelFollower1.getMotorTemperature();
    REVLibError flywheelFollower1LastError = flywheelFollower1.getLastError();
    double flywheelFollower2Temp = flywheelFollower2.getMotorTemperature();
    REVLibError flywheelFollower2LastError = flywheelFollower2.getLastError();
    double hoodTemp = hood.getMotorTemperature();
    REVLibError hoodLastError = hood.getLastError();
    if (flywheelLeaderLastError != REVLibError.kOk
        || flywheelLeaderTemp == 0
        || flywheelFollower1LastError != REVLibError.kOk
        || flywheelFollower1Temp == 0
        || flywheelFollower2LastError != REVLibError.kOk
        || flywheelFollower2Temp == 0
        || hoodLastError != REVLibError.kOk
        || hoodTemp == 0) {
      inputs.shooterStatus = false;
    } else {
      inputs.shooterStatus = true;
    }
  }

  @Override
  public void setVelocityFlywheel(double RPM) {
    flywheelLeaderController.setSetpoint(RPM * ShooterConstants.FLYWHEEL_GEAR_RATIO, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPositionHood(Angle angle) {
    hoodController.setSetpoint(ShooterConstants.kHoodAnglesTable.getOutput(angle.in(Rotations)), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stopHood() {
    hood.set(0);
  }
}
