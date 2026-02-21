package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class GameHubStatus {

  public static boolean isHubActive() {
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (DriverStation.isDisabled()) {
      return false;
    }
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {
      return true;
    }
    char status = gameData.toUpperCase().charAt(0);

    return status == 'A';
  }

  public static void logStatus() {
    Logger.recordOutput("Status/GameHubActive", isHubActive());
  }
}
