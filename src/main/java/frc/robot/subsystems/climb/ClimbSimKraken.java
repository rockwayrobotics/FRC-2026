package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CAN;

public class ClimbSimKraken implements ClimbIO {
  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.001, ClimbConstants.CLIMB_GEAR_RATIO),
          gearbox);
  private final TalonFX motor = new TalonFX(CAN.CLIMB, "rio");
  private TalonFXSimState simMotor = motor.getSimState();
  private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  private double position = 0.0;
  // private double velocityRPM = 0.0;
  // private double kV = 20;

  public ClimbSimKraken() {
    FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs();
    MotorOutputConfigs armMotorOutputConfigs = new MotorOutputConfigs();
    armFeedbackConfigs.SensorToMechanismRatio = ClimbConstants.CLIMB_GEAR_RATIO; // FIXME: Fix Ratio
    armMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    armMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    Slot0Configs PIDConfig = motorConfigs.Slot0;
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
    simMotor.setMotorType(MotorType.KrakenX60);
    motor.getConfigurator().apply(motorConfigs);
    simMotor = motor.getSimState();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    // Update simulation
    motorSim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
    motorSim.update(0.02); // FIXME: This is a fixed 50Hz value

    simMotor.setRawRotorPosition(motorSim.getAngularPositionRotations());
    simMotor.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60.0); // We want RPS
    simMotor.setSupplyVoltage(
        RobotController.getBatteryVoltage()
            - simMotor.getSupplyCurrent() * 0.002); // FIXME: Assume 2mOhm resistance

    // Then read values and populate inputs
    inputs.position = position;
  }

  @Override
  public void setPose(double pose, boolean fast) {
    pose = (Math.PI * 2 * pose);
    // simMotor.setMotorType(MotorType.KrakenX60);
    var foo = new MotionMagicVoltage(0);
    motor.setControl(foo.withPosition(pose));
  }

  @Override
  public void stop() {
    motor.set(0);
  }
}
