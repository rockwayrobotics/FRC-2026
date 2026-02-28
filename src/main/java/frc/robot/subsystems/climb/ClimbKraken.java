package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CAN;

public class ClimbKraken implements ClimbIO {
  private final TalonFX motor = new TalonFX(CAN.CLIMB);
  private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  // FIXME: Add Configs
  // (https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/configs/TalonFXConfigurator.html)
  public ClimbKraken() {
    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs();
    MotorOutputConfigs armMotorOutputConfigs = new MotorOutputConfigs();
    armFeedbackConfigs.SensorToMechanismRatio = ClimbConstants.CLIMB_GEAR_RATIO; // FIXME: Fix Ratio
    armMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    armMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    var PIDConfig = motorConfigs.Slot0;
    PIDConfig.kS = ClimbConstants.KrakenConstants.CLIMB_KS; // FIXME: Actually Tune SVAPID
    PIDConfig.kV = ClimbConstants.KrakenConstants.CLIMB_KV;
    PIDConfig.kA = ClimbConstants.KrakenConstants.CLIMB_KA;
    PIDConfig.kP = ClimbConstants.KrakenConstants.CLIMB_KP;
    PIDConfig.kI = ClimbConstants.KrakenConstants.CLIMB_KI;
    PIDConfig.kD = ClimbConstants.KrakenConstants.CLIMB_KD;
    PIDConfig.GravityType = GravityTypeValue.Elevator_Static;
    PIDConfig.kG = ClimbConstants.KrakenConstants.CLIMB_KG;
    motorConfigs.withFeedback(armFeedbackConfigs);
    motorConfigs.withMotorOutput(armMotorOutputConfigs);
    motor.getConfigurator().apply(motorConfigs);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {}

  @Override
  public void setPos(double pose, boolean fast) {
    // motor.setVoltage(volts);
    final var foo = new MotionMagicVoltage(0);
    motor.setControl(foo.withPosition(pose));
  }

  @Override
  public void stop() {
    motor.set(0);
  }
}
