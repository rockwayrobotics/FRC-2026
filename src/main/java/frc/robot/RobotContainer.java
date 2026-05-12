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

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbNEO2;
import frc.robot.subsystems.climb.ClimbSimNEO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodReal;
import frc.robot.subsystems.hood.HoodSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerReal;
import frc.robot.subsystems.indexer.IndexerSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeKraken;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.intakeExtender.IntakeExtender;
import frc.robot.subsystems.intakeExtender.IntakeExtenderIO;
import frc.robot.subsystems.intakeExtender.IntakeExtenderReal;
import frc.robot.subsystems.intakeExtender.IntakeExtenderSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerReal;
import frc.robot.subsystems.kicker.KickerSim;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
  private final Kicker kicker;
  private final Intake intake;
  private final IntakeExtender intakeExtender;
  private final Shooter shooter;
  private final Hood hood;
  private final Climb climb;

  public static final double JOYSTICK_SCALE = 0.2;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final GenericHID operatorButtonBoard = new GenericHID(2);
  // Dashboard inputs

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    led = new Led();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(DriveConstants.swerveModuleConfigs[0]), // FL
                new ModuleIOSpark(DriveConstants.swerveModuleConfigs[1]), // FR
                new ModuleIOSpark(DriveConstants.swerveModuleConfigs[2]), // BL
                new ModuleIOSpark(DriveConstants.swerveModuleConfigs[3])); // BR
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera_front, robotToCameraFront),
                new VisionIOPhotonVision(camera_back, robotToCameraBack));

        indexer = new Indexer(new IndexerReal());
        kicker = new Kicker(new KickerReal());
        intake = new Intake(new IntakeKraken()); // IntakeReal());
        intakeExtender = new IntakeExtender(new IntakeExtenderReal()); // IntakeExtenderReal());
        shooter = new Shooter(new ShooterReal());
        hood = new Hood(new HoodReal());
        climb = new Climb(new ClimbNEO2());
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
        kicker = new Kicker(new KickerSim());
        intake = new Intake(new IntakeSim());
        intakeExtender = new IntakeExtender(new IntakeExtenderSim());
        shooter = new Shooter(new ShooterSim());
        hood = new Hood(new HoodSim());
        climb = new Climb(new ClimbSimNEO());
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
        kicker = new Kicker(new KickerIO() {});
        intake = new Intake(new IntakeIO() {});
        intakeExtender = new IntakeExtender(new IntakeExtenderIO() {});
        shooter = new Shooter(new ShooterIO() {});
        hood = new Hood(new HoodIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // Configure the button bindings
    configureButtonBindings();

    // Get PathPlanner ready to avoid Java startup time.
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  public void disabledInit() {
    intakeExtender.enableCoastMode();
  }

  public void enabledInit() {
    intakeExtender.enableBrakeMode();
    drive.enableBrakeMode();
  }

  private void configureDefaultCommands() {
    // Default command, normal field-relative drive
    // Strafe
    // Rotate

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> controller.rightBumper().getAsBoolean(),
            () -> controller.leftBumper().getAsBoolean()));

    // Stop indexer by default
    indexer.setDefaultCommand(
        Commands.run(
            () -> {
              if (!DriverStation.isAutonomous()) {
                indexer.stop();
              }
            },
            indexer));

    // Stop kicker by default
    kicker.setDefaultCommand(
        Commands.run(
            () -> {
              if (!DriverStation.isAutonomous()) {
                kicker.stop();
              }
            },
            kicker));

    // Stop shooter by default (coast mode flywheel)
    shooter.setDefaultCommand(
        Commands.run(
            () -> {
              if (!DriverStation.isAutonomous() && !shooter.isOperatorOverriding()) {
                shooter.stop();
              }
            },
            shooter));

    // Retract hood by default
    hood.setDefaultCommand(
        Commands.run(
            () -> {
              if (!DriverStation.isAutonomous() && !hood.isOperatorOverriding()) {
                hood.stop();
              }
            },
            hood));

    // Retract intake by default
    // intakeExtender.setDefaultCommand(IntakeCommands.autoRetract(intakeExtender));
    intakeExtender.setDefaultCommand(
        Commands.run(
            () -> {
              if (!DriverStation.isAutonomous()) {
                intakeExtender.stop();
              }
            },
            intakeExtender));

    // Stop intake rollers by default
    intake.setDefaultCommand(
        Commands.run(
            () -> {
              if (!DriverStation.isAutonomous()) {
                intake.intake(0.0);
              }
            },
            intake));

    // Stop climb by default
    climb.setDefaultCommand(Commands.run(() -> climb.stop(), climb));
  }

  private void configureDriverCommands() {
    controller
        .leftTrigger()
        .whileTrue(ShooterCommands.magicTrigger(shooter, hood, drive, controller));
    controller.x().whileTrue(ShooterCommands.dumpShort(shooter, hood, controller));

    controller
        .back()
        .whileTrue(IntakeCommands.intakeManual(intake, IntakeConstants.ROLLER_EJECT_DUTY_CYCLE));

    controller
        .y()
        .whileTrue(IntakeCommands.intakeManual(intake, IntakeConstants.ROLLER_DUTY_CYCLE));
    controller.y().whileTrue(IndexerCommands.agitate(indexer, kicker));
    controller.rightBumper().onTrue(IntakeCommands.extend(intakeExtender));
    controller.rightBumper().onFalse(IntakeCommands.retract(intakeExtender));

    controller.a().whileTrue(IntakeCommands.trashCompact(intakeExtender, intake));
    controller.b().whileTrue(ShooterCommands.againstHubShot(shooter, hood, controller));
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                      drive.resetEncoders();
                    },
                    drive)
                .ignoringDisable(true));

    controller.povLeft().whileTrue(Commands.run(drive::stopWithX, drive));
    controller.povRight().whileTrue(Commands.run(drive::stopWithX, drive));

    // Shoot Sequence
    controller
        .rightTrigger()
        .whileTrue(Commands.parallel(IndexerCommands.feedShooter(indexer, kicker)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Controller notes:
    // // Full speed all the time
    // // Left bumper to drive while slower (50%)
    // Left trigger - hold down to spin up flywheel, point at hub and set hood
    // angle. Still be able
    // to drive
    // Right trigger - hold down to run the auger + kickers
    // Driver has auto-shoot triggers, driver commands take precedence
    // Operator has both semi-manual setpoints
    // and also manual move the setpoint up or down for everything.
    // Operator needs joysticks for moving rpm setpoints gradually
    // Hood default position is DOWN
    // operator may have manual control of that too
    // toggle situation where the operator takes over manual control and the hood
    // doesn't auto-fold.
    // but if the driver ever auto-shoots after that, then the hood will auto-fold.
    // Driver has controls for extend and start spinning intakes,
    // Add agitate button for operator (run kicker backwards, auger forwards)
    // May want to add this agitate sequence to be part of intaking as well.
    configureDefaultCommands();
    configureDriverCommands();

    /*
     * // Sad, infinite weight, infinite LEDs
     * LEDPattern base =
     * LEDPattern.progressMaskLayer(
     * () -> {
     * return 1.0 - (Math.abs(drive.getRotation().getDegrees()) / 180.0);
     * });
     * LEDPattern rainbow = LEDPattern.solid(Color.kBlue);
     * controller.povUp().whileTrue(led.runPattern(rainbow.mask(base)));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public Pose2d getPose() {
    return drive.getPose();
  }
}
