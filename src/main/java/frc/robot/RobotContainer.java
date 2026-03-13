// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.vision.VisionConstants.camera_back;
import static frc.robot.subsystems.vision.VisionConstants.camera_front;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraBack;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraFront;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbNEO2;
import frc.robot.subsystems.climb.ClimbSimKraken;
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
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.intakeExtender.IntakeExtender;
import frc.robot.subsystems.intakeExtender.IntakeExtenderIO;
import frc.robot.subsystems.intakeExtender.IntakeExtenderReal;
import frc.robot.subsystems.intakeExtender.IntakeExtenderSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
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
  private final Kicker kicker;
  private final Intake intake;
  private final IntakeExtender intakeExtender;
  private final Shooter shooter;
  private final Hood hood;
  private final Climb climb;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final GenericHID operatorButtonBoard = new GenericHID(2);
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
        intake = new Intake(new IntakeReal());
        intakeExtender = new IntakeExtender(new IntakeExtenderReal());
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
        kicker = new Kicker(new KickerIO() {});
        intake = new Intake(new IntakeIO() {});
        intakeExtender = new IntakeExtender(new IntakeExtenderIO() {});
        shooter = new Shooter(new ShooterIO() {});
        hood = new Hood(new HoodIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // Pathplanner Named Commands
    // NamedCommands.registerCommand("SetupShotFor2",(ShooterCommands.definitiveShoot(shooter,
    // hood,
    // 3556.33588,20)));
    NamedCommands.registerCommand(
        "StopShooter",
        Commands.runOnce(
            () -> {
              shooter.stop();
              hood.stop();
            },
            shooter,
            hood));
    NamedCommands.registerCommand(
        "StopIndexer",
        Commands.runOnce(
            () -> {
              indexer.stop();
              kicker.stop();
            },
            indexer,
            kicker));
    NamedCommands.registerCommand(
        "SetupShootHub", ShooterCommands.definitiveShoot(shooter, hood, 3200, 15));
    NamedCommands.registerCommand(
        "Shoot2", IndexerCommands.feedShooter(indexer, kicker).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand(
        "SetupShotFor2", ShooterCommands.definitiveShoot(shooter, hood, 3556.33588, 20));
    NamedCommands.registerCommand(
        "ExtendIntake", IntakeCommands.extend(intakeExtender).withTimeout(Seconds.of(1)));
    NamedCommands.registerCommand(
        "RetractIntake", IntakeCommands.retract(intakeExtender).withTimeout(Seconds.of(1)));
    NamedCommands.registerCommand(
        "IntakeShortTime",
        Commands.sequence(
            IntakeCommands.intakeManual(intake, IntakeConstants.ROLLER_DUTY_CYCLE)
                .withTimeout(Seconds.of(2)),
            IntakeCommands.intakeManual(intake, 0.0).withTimeout(Seconds.of(0.1))));
    NamedCommands.registerCommand(
        "SetupHubShot",
        ShooterCommands.setupHubShot(
            shooter, hood, drive, () -> 0.0, () -> 0.0, () -> false, () -> false));

    new EventTrigger("IntakeZone")
        .onTrue(NamedCommands.getCommand("ExtendIntake"))
        .whileTrue(
            Commands.sequence(
                Commands.waitUntil(() -> intakeExtender.getExtendAngle() > 30),
                IntakeCommands.intakeManual(intake, IntakeConstants.ROLLER_DUTY_CYCLE)))
        .onFalse(
            Commands.parallel(
                IntakeCommands.intakeManual(intake, 0), NamedCommands.getCommand("RetractIntake")));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Autos", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Center Shoot Hub",
        Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    drive.setPose(new Pose2d(new Translation2d(3.5, 4), Rotation2d.k180deg));
                  } else {
                    drive.setPose(new Pose2d(new Translation2d(13, 4), Rotation2d.kZero));
                  }
                }),
            ShooterCommands.definitiveShoot(shooter, hood, 3400, 15),
            Commands.waitUntil(() -> shooter.atFlywheelSetpoint(100)),
            NamedCommands.getCommand("Shoot2"),
            NamedCommands.getCommand("Stop")));

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());

    // Configure the button bindings
    configureButtonBindings();

    // Get PathPlanner ready to avoid Java startup time.
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
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
    intakeExtender.setDefaultCommand(Commands.run(() -> intakeExtender.stop(), intakeExtender));

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
    (operatorController
            .a()
            .or(operatorController.b())
            .or(operatorController.x())
            .or(operatorController.y()))
        .and(controller.leftTrigger())
        .whileTrue(ShooterCommands.activateDeferredHood(hood));

    (operatorController
            .a()
            .or(operatorController.b())
            .or(operatorController.x())
            .or(operatorController.y()))
        .negate()
        .and(controller.leftTrigger())
        .whileTrue(
            ShooterCommands.setupHubShot(
                shooter,
                hood,
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> controller.rightBumper().getAsBoolean(),
                () -> controller.leftBumper().getAsBoolean()));
    // .whileTrue(ShooterCommands.testShoot(shooter, hood));

    controller.povLeft().whileTrue(Commands.run(drive::stopWithX, drive));
    controller.povRight().whileTrue(Commands.run(drive::stopWithX, drive));

    controller.x().onTrue(NamedCommands.getCommand("ExtendIntake"));
    controller.a().onTrue(NamedCommands.getCommand("RetractIntake"));
    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDrivePointAtHub(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    // Point at Hub
    // controller
    // .leftTrigger()
    // .and(controller.rightBumper().negate())
    // .whileTrue(ShooterCommands.testShoot(shooter, hood));
    // ShooterCommands.aimOnMove(
    // shooter, drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // Point at Hub while slow down
    // controller
    // .rightBumper()
    // .and(controller.leftTrigger())
    // .whileTrue(
    // ShooterCommands.aimOnMove(
    // shooter,
    // drive,
    // () -> -controller.getLeftY() * SLOW_SPEED,
    // () -> -controller.getLeftX() * SLOW_SPEED));

    // Shoot Sequence
    controller.rightTrigger().whileTrue(IndexerCommands.feedShooter(indexer, kicker));

    // X-Stop
    // FIXME: Change this to D-M1 and D-M2??
    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    // Auto-Intake
    // controller.leftBumper().whileTrue(IntakeCommands.autoIntake(intakeExtender,
    // intake));

    controller
        .povUp()
        .whileTrue(
            Commands.run(
                () -> {
                  climb.dutyCycle(0.5);
                },
                climb));

    controller
        .povDown()
        .whileTrue(
            Commands.run(
                () -> {
                  climb.dutyCycle(-0.5);
                },
                climb));

    //////////// Testing commands below here ///////////////////////

    // FIXME: Probably remove this after shot testing
    /*
     * controller.a().whileTrue(TestCommands.testShot(shooter, hood, indexer,
     * kicker));
     *
     * controller.rightBumper().whileTrue(Commands.run(() ->
     * indexer.setVelocityKicker(100), indexer));
     *
     * controller
     * .y()
     * .whileTrue(
     * DriveCommands.joystickDrivePointAtHub(
     * drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
     * controller
     * .leftTrigger()
     * .whileTrue(
     * Commands.parallel(
     * ShooterCommands.aimOnMove(
     * shooter, drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
     * IndexerCommands.feedShooterFancy(indexer, shooter, drive)));
     *
     * controller.povLeft().whileTrue(DriveCommands.turnSetpoint(drive,
     * Rotation2d.kCCW_90deg));
     * controller.povRight().whileTrue(DriveCommands.turnSetpoint(drive,
     * Rotation2d.kPi));
     */
  }

  private void configureOperatorCommands() {
    // Extend
    operatorController.rightTrigger().onTrue(IntakeCommands.extend(intakeExtender));
    // Retract
    operatorController.rightBumper().onTrue(IntakeCommands.retract(intakeExtender));

    // Complete fold - DISABLED
    /*
     * double lastRBPressTime = 0.0;
     * operatorController
     * .rightBumper()
     * .onTrue(
     * Commands.runOnce(
     * () -> {
     * double now = Timer.getFPGATimestamp();
     * if (lastRBPressTime > 0.0 && now - lastRBPressTime < 0.3) {
     * // Double press, spit out balls for 3 seconds while folding
     * CommandScheduler.getInstance()
     * .schedule(IntakeCommands.ejectBalls(intakeExtender, intake));
     * }
     * },
     * intake));
     */

    // Manual extend
    new JoystickButton(operatorButtonBoard, 5)
        .whileTrue(IntakeCommands.extendManual(intakeExtender, 0.2));
    // Manual retract
    new JoystickButton(operatorButtonBoard, 7)
        .whileTrue(IntakeCommands.extendManual(intakeExtender, -0.2));

    // Manual forward
    new JoystickButton(operatorButtonBoard, 8)
        .whileTrue(IntakeCommands.intakeManual(intake, IntakeConstants.ROLLER_DUTY_CYCLE));
    // Manual reverse
    new JoystickButton(operatorButtonBoard, 6)
        .whileTrue(IntakeCommands.intakeManual(intake, -IntakeConstants.ROLLER_DUTY_CYCLE));

    // Intake balls
    operatorController
        .leftTrigger()
        .whileTrue(IntakeCommands.intakeManual(intake, IntakeConstants.ROLLER_DUTY_CYCLE));
    operatorController.leftTrigger().whileTrue(IndexerCommands.agitate(indexer, kicker));
    operatorController
        .leftBumper()
        .whileTrue(IntakeCommands.intakeManual(intake, IntakeConstants.ROLLER_EJECT_DUTY_CYCLE));

    // Agitate
    // operatorController.a().whileTrue(IndexerCommands.agitate(indexer, kicker));
    // Unjam - DISABLED
    // operatorController.b().whileTrue(IndexerCommands.unjam(indexer, kicker));

    operatorController.a().whileTrue(ShooterCommands.cornerSetpointShoot(shooter, hood));
    operatorController.b().whileTrue(ShooterCommands.trenchSetpointShoot(shooter, hood));
    operatorController.x().whileTrue(ShooterCommands.sideTowerSetpointShoot(shooter, hood));
    operatorController.y().whileTrue(ShooterCommands.towerSetpointShoot(shooter, hood));

    // operatorController.leftTrigger().whileTrue(ShooterCommands.activateDeferredHood(hood));

    // Forward Augers
    new JoystickButton(operatorButtonBoard, 3).whileTrue(IndexerCommands.augersFeed(indexer));
    // Reverse Augers
    new JoystickButton(operatorButtonBoard, 4).whileTrue(IndexerCommands.augersReverse(indexer));

    // Disabled configure PID for shooter:
    // operatorController.back().onTrue(ShooterCommands.configureLeader(shooter));

    // XBox controller axes:
    // 0: left stick X
    // 1: left stick Y
    // 2: left trigger
    // 3: right trigger
    // 4: right stick X
    // 5: right stick Y
    // Y-axes are inverted, so pushing stick forward gives negative value.

    // Manual spin up
    // Do something with
    // operatorController.getRightY();
    operatorController
        .axisLessThan(5, -0.1) // Axis is negated, so this is up.
        .onTrue(
            Commands.runOnce(
                () -> {
                  // Set state so that flywheel maintains speed and doesn't stop.
                  shooter.setOperatorOverride(true);
                }));

    operatorController
        .axisMagnitudeGreaterThan(5, 0.01) // -0.01 - 0.01 deadzone
        .whileTrue(ShooterCommands.manualFlywheel(shooter, () -> -operatorController.getRightY()));

    // Manual hood pivot
    // Do something with
    // operatorController.getLeftY();
    operatorController
        .axisMagnitudeGreaterThan(1, 0.1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  // Set state so that hood maintains position and doesn't fold.
                  hood.setOperatorOverride(true);
                }));
    operatorController
        .axisMagnitudeGreaterThan(1, 0.01) // -0.01 - 0.01 deadzone
        .whileTrue(ShooterCommands.manualHood(hood, () -> -operatorController.getLeftY()));

    /*
     * operatorController
     * .axisMagnitudeGreaterThan(1, 0.1)
     * .whileTrue(
     * Commands.run(
     * () -> {
     * System.out.println("Left stick on: " + -operatorController.getLeftY());
     * }));
     */

    // Manual kicker forward
    new JoystickButton(operatorButtonBoard, 2)
        .whileTrue(
            Commands.run(
                () -> {
                  kicker.setVelocityKicker(KickerConstants.KICKER_FEED_RPM);
                },
                kicker));
    // Reverse kicker
    new JoystickButton(operatorButtonBoard, 1)
        .whileTrue(
            Commands.run(
                () -> {
                  kicker.setVelocityKicker(KickerConstants.KICKER_AGITATE_RPM);
                },
                kicker));

    ////////////////////// Testing commands below here ///////////////////////

    /*
     * operatorController.b().whileTrue(Commands.run(() -> indexer.augersFeed(),
     * indexer));
     * operatorController.x().whileTrue(Commands.run(() -> indexer.augersReverse(),
     * indexer));
     * operatorController.x().whileTrue(Commands.run(() ->
     * indexer.setVelocityKicker(100), indexer));
     *
     * operatorController.povDown().onTrue(Commands.runOnce(() -> climb.unclimb(),
     * climb));
     * operatorController.povUp().onTrue(Commands.runOnce(() -> climb.climb(),
     * climb));
     * operatorController.povRight().onTrue(Commands.runOnce(() -> climb.extend(),
     * climb));
     * operatorController.povLeft().onTrue(Commands.runOnce(() -> climb.retract(),
     * climb));
     *
     * operatorController.a().whileTrue(Commands.run(() ->
     * shooter.setVelocityFlywheel(200), shooter));
     * operatorController
     * .y()
     * .whileTrue(
     * Commands.run(() -> shooter.setPositionHood(Angle.ofBaseUnits(0, Degrees)),
     * shooter));
     * operatorController
     * .povCenter()
     * .whileTrue(
     * Commands.run(() -> shooter.setPositionHood(Angle.ofBaseUnits(30, Degrees)),
     * shooter));
     */
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
    configureOperatorCommands();

    /*
     * // Sad, no weight, no LEDs
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
    return autoChooser.get();
  }

  public Pose2d getPose() {
    return drive.getPose();
  }
}
