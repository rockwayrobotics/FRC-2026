package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
  private static final int port = 0;
  private static final int length = 120;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public Led() {
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.start();

    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  @Override
  public void periodic() {
    led.setData(buffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  } // FIXME remove

  public void setColour(Color c) {
    LEDPattern solid = LEDPattern.solid(c);
    solid.applyTo(buffer);
  }

  public void setColourRGB(double r, double g, double b) {
    setColour(new Color(r, g, b));
  }
}
