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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
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
  private final Intake intake;
  private final IntakeExtender intakeExtender;
  private final Shooter shooter;
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
                new VisionIOPhotonVision(camera_front, robotToCameraFront) /*
                                                                                    * ,
                                                                                    * new
                                                                                    * VisionIOPhotonVision(camera_back,
                                                                                    * robotToCameraBack)
                                                                                    */);

        indexer = new Indexer(new IndexerReal());
        intake = new Intake(new IntakeReal());
        intakeExtender = new IntakeExtender(new IntakeExtenderReal());
        shooter = new Shooter(new ShooterReal());
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
        intake = new Intake(new IntakeSim());
        intakeExtender = new IntakeExtender(new IntakeExtenderSim());
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
        intakeExtender = new IntakeExtender(new IntakeExtenderIO() {});
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
    // Strafe
    // Rotate
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Stop indexer by default
    indexer.setDefaultCommand(Commands.run(() -> indexer.stop(), indexer));

    // Stop shooter by default (coast mode flywheel)
    // This also retracts hood
    shooter.setDefaultCommand(Commands.run(() -> shooter.stop(), shooter));

    // Retract intake by default
    intakeExtender.setDefaultCommand(IntakeCommands.autoRetract(intakeExtender));

    // Stop intake rollers by default
    intake.setDefaultCommand(IntakeCommands.intakeManual(intake, 0.0));

    // Stop climb by default
    climb.setDefaultCommand(Commands.run(() -> climb.stop(), climb));
  }

  private void configureDriverCommands() {
    // Slow Down
    // Left bumper is 50% speed
    final double SLOW_SPEED = 0.5;
    controller
        .rightBumper()
        // .and(controller.leftTrigger().negate())
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY() * SLOW_SPEED,
                () -> -controller.getLeftX() * SLOW_SPEED,
                () -> -controller.getRightX() * SLOW_SPEED));

    // Point at Hub
    controller
        .leftTrigger()
        .and(controller.rightBumper().negate())
        .whileTrue(ShooterCommands.testShoot(shooter));
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
    controller.rightTrigger().whileTrue(IndexerCommands.feedShooter(indexer));

    // X-Stop
    // FIXME: Change this to D-M1 and D-M2??
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

    // Auto-Intake
    controller.leftBumper().whileTrue(IntakeCommands.autoIntake(intakeExtender, intake));

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
     * ontroller.a().whileTrue(TestCommands.testShot(shooter, indexer));
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
    operatorController.leftBumper().onTrue(IntakeCommands.extend(intakeExtender));
    // Retract
    operatorController.rightBumper().onTrue(IntakeCommands.retract(intakeExtender));

    // Complete fold
    double lastRBPressTime = 0.0;
    operatorController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double now = Timer.getFPGATimestamp();
                  if (lastRBPressTime > 0.0 && now - lastRBPressTime < 0.3) {
                    // Double press, spit out balls for 3 seconds while folding
                    CommandScheduler.getInstance()
                        .schedule(IntakeCommands.ejectBalls(intakeExtender, intake));
                  }
                },
                intake));

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
    operatorController.leftTrigger().whileTrue(IndexerCommands.agitate(indexer));

    // Agitate
    operatorController.a().whileTrue(IndexerCommands.agitate(indexer));
    // Unjam
    operatorController.b().whileTrue(IndexerCommands.unjam(indexer));

    // Forward Augers
    new JoystickButton(operatorButtonBoard, 3).whileTrue(IndexerCommands.augersFeed(indexer));
    // Reverse Augers
    new JoystickButton(operatorButtonBoard, 4).whileTrue(IndexerCommands.augersReverse(indexer));

    // Manual spin up
    // Do something with
    // operatorController.getRightY();

    // Manual hood pivot
    // Do something with
    // operatorController.getLeftY();

    // Manual kicker forward
    new JoystickButton(operatorButtonBoard, 2)
        .onTrue(IndexerCommands.kickerVelocity(indexer, IndexerConstants.KICKER_FEED_RPM));
    // Reverse kicker
    new JoystickButton(operatorButtonBoard, 1)
        .onTrue(IndexerCommands.kickerVelocity(indexer, IndexerConstants.KICKER_AGITATE_RPM));

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
}
