package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
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
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.CAN;
import frc.robot.util.SparkUtil;

public class HoodReal implements HoodIO {
  private final SparkBase hood = new SparkMax(CAN.VARIABLE_HOOD, MotorType.kBrushless);

  private final AbsoluteEncoder hoodEncoder = hood.getAbsoluteEncoder();
  private final SparkClosedLoopController hoodController = hood.getClosedLoopController();

  public HoodReal() {
    var hoodConfig = new SparkMaxConfig();
    hoodConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(1)
        .voltageCompensation(12.0);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .maxOutput(0.05)
        .minOutput(-0.05)
        .pid(HoodConstants.HOOD_KP, HoodConstants.HOOD_KI, HoodConstants.HOOD_KD);
    hoodConfig.absoluteEncoder.positionConversionFactor(
        HoodConstants.HOOD_ENCODER_POSITION_CONVERSION_FACTOR);
    hoodConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    hoodConfig
        .softLimit
        .forwardSoftLimit(HoodConstants.HOOD_INTERNAL_FORWARD_LIMIT) // Test forward -> 50
        .reverseSoftLimit(HoodConstants.HOOD_INTERNAL_REVERSE_LIMIT)
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
  public void updateInputs(HoodIOInputs inputs) {
    SparkUtil.ifOk(
        hood,
        hoodEncoder::getPosition,
        (value) -> {
          inputs.hoodRawPosition = value;
          inputs.hoodPosition = HoodConstants.kHoodAnglesTable.inverseGet(value);
        });

    double hoodTemp = hood.getMotorTemperature();
    REVLibError hoodLastError = hood.getLastError();
    if (hoodLastError != REVLibError.kOk || hoodTemp == 0) {
      inputs.hoodStatus = false;
    } else {
      inputs.hoodStatus = true;
    }
  }

  @Override
  public void setPositionHood(Angle angle) {
    hoodController.setSetpoint(
        HoodConstants.kHoodAnglesTable.getOutput(angle.in(Degrees)),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stopHood() {
    setPositionHood(Degrees.of(HoodConstants.HOOD_REST_POINT));
  }
}
