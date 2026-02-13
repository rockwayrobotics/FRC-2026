package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CAN;

public class ClimbReal implements ClimbIO {
  private final TalonFX motor = new TalonFX(CAN.CLIMB);

  public ClimbReal() {}

  @Override
  public void updateInputs(ClimbIOInputs inputs) {}
}
