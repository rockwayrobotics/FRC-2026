// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera_back;
import static frc.robot.subsystems.vision.VisionConstants.camera_front;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraBack;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraFront;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.shooterCommands.AimOnMove;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbSimKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.ModuleIOSparkAbsolute;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Led led;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final Climb climb;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    led = new Led();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSparkAbsolute(DriveConstants.swerveModuleConfigsDev[0]), // FL
                new ModuleIOSpark(DriveConstants.swerveModuleConfigsDev[1]), // FR
                new ModuleIOSpark(DriveConstants.swerveModuleConfigsDev[2]), // BL
                new ModuleIOSpark(DriveConstants.swerveModuleConfigsDev[3])); // BR
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera_front, robotToCameraFront),
                new VisionIOPhotonVision(camera_back, robotToCameraBack));

        indexer = new Indexer(new IndexerSim()); // FIXME: Simulated indexer for now until connected
        intake = new Intake(new IntakeSim()); // same as ^
        shooter = new Shooter(new ShooterSim());
        climb = new Climb(new ClimbSimKraken());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera_front, robotToCameraFront, drive::getPose),
                new VisionIOPhotonVisionSim(camera_back, robotToCameraBack, drive::getPose));

        indexer = new Indexer(new IndexerSim());
        intake = new Intake(new IntakeSim());
        shooter = new Shooter(new ShooterSim());
        climb = new Climb(new ClimbSimKraken());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        indexer = new Indexer(new IndexerIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Autos", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // FIXME: Default command is to stop the indexer. If we want them to run
    // continuously,
    // then we should have a command for that which is scheduled.
    indexer.setDefaultCommand(Commands.run(() -> indexer.stop(), indexer));

    // There is no default climb command so that it continues to go to its setpoint.
  }

  private void configureOperatorCommands() {
    operatorController.b().whileTrue(Commands.run(() -> indexer.augersFeed(), indexer));
    operatorController.x().whileTrue(Commands.run(() -> indexer.augersReverse(), indexer));

    operatorController.povDown().onTrue(Commands.runOnce(() -> climb.unclimb(), climb));
    operatorController.povUp().onTrue(Commands.runOnce(() -> climb.climb(), climb));
    operatorController.povRight().onTrue(Commands.runOnce(() -> climb.extend(), climb));
    operatorController.povLeft().onTrue(Commands.runOnce(() -> climb.retract(), climb));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDefaultCommands();
    configureOperatorCommands();

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDrivePointAtHub(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    controller
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                AimOnMove.run(
                    shooter, drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                IndexerCommands.feedShooter(indexer, shooter, drive)));

    controller.povLeft().whileTrue(DriveCommands.turnSetpoint(drive, Rotation2d.kCCW_90deg));
    controller.povRight().whileTrue(DriveCommands.turnSetpoint(drive, Rotation2d.kPi));
    LEDPattern base =
        LEDPattern.progressMaskLayer(
            () -> {
              return 1.0 - (Math.abs(drive.getRotation().getDegrees()) / 180.0);
            });
    LEDPattern rainbow = LEDPattern.solid(Color.kBlue);
    controller.povUp().whileTrue(led.runPattern(rainbow.mask(base)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
