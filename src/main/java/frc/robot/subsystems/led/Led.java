package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IndexerCommands;
import frc.robot.util.GameHubStatus;

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
    if (DriverStation.isAutonomous()) {
      double timeLeft = Timer.getMatchTime();

      // 1. Priority: Auto Ending (Last 5 seconds)
      if (timeLeft < 5.0 && timeLeft > 0) {
        this.setEndStrobe(Color.kGold);
      } else if (GameHubStatus.isHubActive()) {
        this.setColor(Color.kCyan); // Hub Active
        if (IndexerCommands.isShooting) {
          this.setColor(Color.kDarkCyan); // Shooting
        }
      } else {
        this.setColor(Color.kMagenta); // Inactive
      }
    }
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

  public void setColor(Color color) {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
    led.setData(buffer);
  }

  /** Flashes the LEDs between a color and Black. 0.1s on, 0.1s off (adjust the 0.2 for speed) */
  public void setStrobe(Color color) {
    if ((Timer.getFPGATimestamp() % 0.2) > 0.1) {
      setColor(color);
    } else {
      setColor(Color.kBlack); // "Off" state
    }
  }

  public void setEndStrobe(Color color) {
    if ((Timer.getFPGATimestamp() % 0.2) > 0.1) {
      setColor(color);
    } else {
      setColor(Color.kBlack); // "Off" state
    }
  }
}
