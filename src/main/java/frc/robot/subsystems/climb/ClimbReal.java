package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CAN;

public class ClimbReal implements ClimbIO {
  private final TalonFX motor = new TalonFX(CAN.CLIMB);
  private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  // FIXME - Add Configs
  // (https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/configs/TalonFXConfigurator.html)
  public ClimbReal() {
    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs();
    MotorOutputConfigs armMotorOutputConfigs = new MotorOutputConfigs();
    armFeedbackConfigs.SensorToMechanismRatio = 45; // FIXME - Correct Ratio
    armMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    armMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    var PIDConfig = motorConfigs.Slot0;
    PIDConfig.kS = 0.21; // FIXME - Actually Tune SVAPID
    PIDConfig.kV = 3.2;
    PIDConfig.kA = 0.0;
    PIDConfig.kP = 60;
    PIDConfig.kI = 0;
    PIDConfig.kD = 2.0;
    PIDConfig.GravityType = GravityTypeValue.Elevator_Static;
    PIDConfig.kG = 0.4;
    motorConfigs.withFeedback(armFeedbackConfigs);
    motorConfigs.withMotorOutput(armMotorOutputConfigs);
    motor.getConfigurator().apply(motorConfigs);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
