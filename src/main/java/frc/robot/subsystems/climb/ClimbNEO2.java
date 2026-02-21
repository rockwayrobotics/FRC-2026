package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Millimeters;

import com.revrobotics.PersistMode;
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
import frc.robot.subsystems.climb.ClimbConstants.NEO2Constants;
import frc.robot.util.SparkUtil;

public class ClimbNEO2 implements ClimbIO {
  private final SparkBase motor = new SparkMax(CAN.CLIMB, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  private static final ClosedLoopSlot fastSlot = ClosedLoopSlot.kSlot0;
  private static final ClosedLoopSlot slowSlot = ClosedLoopSlot.kSlot1;

  public ClimbNEO2() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    // Convert wheel revolutions to mm
    config.encoder.positionConversionFactor(
        ClimbConstants.CLIMB_SPOOL_DIAMETER.in(Millimeters)
            * Math.PI
            / ClimbConstants.CLIMB_GEAR_RATIO);
    config.closedLoop.feedForward.kS(NEO2Constants.CLIMB_KS).kG(NEO2Constants.CLIMB_KG);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(NEO2Constants.CLIMB_KP, 0.0, 0.0)
        .outputRange(-1, 1, fastSlot)
        .outputRange(-0.3, 0.3, slowSlot);
    config.signals.primaryEncoderPositionAlwaysOn(true);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    SparkUtil.ifOk(motor, encoder::getPosition, (value) -> inputs.position = value);
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  public void setPose(double pose, boolean fast) {
    if (fast) {
      controller.setSetpoint(pose, ControlType.kPosition, fastSlot);
    } else {
      controller.setSetpoint(pose, ControlType.kPosition, slowSlot);
    }
  }
}
