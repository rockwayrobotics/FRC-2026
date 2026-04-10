package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CAN;
import java.util.function.Supplier;

public class IntakeKraken implements IntakeIO {
  private final TalonFX motor = new TalonFX(CAN.INTAKE_ROLLERS);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeKraken() {
    // Configure drive motor
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  /** Attempts to run the command until no error is produced. */
  private void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh all signals
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);

    // Update drive inputs
    inputs.intakeStatus = connectedDebounce.calculate(status.isOK());
    inputs.rollerVelocity = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.rollerAppliedVolts = appliedVolts.getValueAsDouble();
    inputs.rollerCurrent = current.getValueAsDouble();
  }

  @Override
  public void intake(double dutyCycle) {
    motor.set(dutyCycle);
  }
}
