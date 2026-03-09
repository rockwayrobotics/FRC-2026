package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double hoodPosition = 0.0; // degrees, 15-45 range
    public double hoodRawPosition =
        0.0; // encoder units, scaled into the 0-360 range but really more like 0-50
    public boolean hoodStatus = false;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPositionHood(Angle angle) {}

  public default void stopHood() {}
}
