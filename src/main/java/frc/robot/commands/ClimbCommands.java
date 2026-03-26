package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {
  public static Command leftAutoLeftClimb(Climb climb) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath setupPath = PathPlannerPath.fromPathFile("Left Trench to Climb");
      PathPlannerPath preClimbPath = PathPlannerPath.fromPathFile("Teleop Left PreClimb");
      PathPlannerPath climbPath = PathPlannerPath.fromPathFile("Auto Left Climb");

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return Commands.sequence(
          Commands.parallel(
              Commands.run(
                      () -> {
                        climb.exten
                        d();
                      },
                      climb)
                  .until(() -> climb.atSetpoint(2)),
              Commands.sequence(
                  AutoBuilder.followPath(setupPath), AutoBuilder.followPath(preClimbPath))),
          AutoBuilder.followPath(climbPath),
          Commands.run(
                  () -> {
                    climb.climb();
                  })
              .until(() -> climb.atSetpoint(2)));

    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }

  public static Command centerLeftClimb(Climb climb) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath setupPath = PathPlannerPath.fromPathFile("Center to Left Climb");
      PathPlannerPath preClimbPath = PathPlannerPath.fromPathFile("Teleop Left PreClimb");
      PathPlannerPath climbPath = PathPlannerPath.fromPathFile("Auto Left Climb");

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return Commands.sequence(
          Commands.parallel(
              Commands.run(
                      () -> {
                        climb.extend();
                      },
                      climb)
                  .until(() -> climb.atSetpoint(2)),
              Commands.sequence(
                  AutoBuilder.followPath(setupPath), AutoBuilder.followPath(preClimbPath))),
          AutoBuilder.followPath(climbPath),
          Commands.run(
                  () -> {
                    climb.climb();
                  })
              .until(() -> climb.atSetpoint(2)));

    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }

  public static Command leftClimb(Climb climb) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath preClimbPath = PathPlannerPath.fromPathFile("Teleop Left PreClimb");
      PathPlannerPath climbPath = PathPlannerPath.fromPathFile("Teleop Left Climb");

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return Commands.sequence(
          Commands.parallel(
              Commands.run(
                      () -> {
                        climb.extend();
                      },
                      climb)
                  .until(() -> climb.atSetpoint(2)),
              AutoBuilder.followPath(preClimbPath)),
          AutoBuilder.followPath(climbPath),
          Commands.run(
                  () -> {
                    climb.climb();
                  })
              .until(() -> climb.atSetpoint(2)));

    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }

  public static Command rightClimb(Climb climb) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath preClimbPath = PathPlannerPath.fromPathFile("Teleop Right PreClimb");
      PathPlannerPath climbPath = PathPlannerPath.fromPathFile("Teleop Right Climb");

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return Commands.sequence(
          Commands.parallel(
              Commands.run(
                      () -> {
                        climb.extend();
                      },
                      climb)
                  .until(() -> climb.atSetpoint(2)),
              AutoBuilder.followPath(preClimbPath)),
          AutoBuilder.followPath(climbPath));
      // Commands.run(
      // () -> {
      // climb.climb();
      // })
      // .until(() -> climb.atSetpoint(2)));

    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }
}
