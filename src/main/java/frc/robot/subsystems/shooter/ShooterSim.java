package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterSim implements ShooterIO {
  // private final FlywheelSim flywheelSim;
  // private final SingleJointedArmSim hoodSim;

  // private final PIDController flywheelPID = new PIDController(0.0004, 0, 0);
  // private final PIDController hoodPID = new PIDController(12.0, 0, 0);

  // private double flywheelSetpointRPM = 0.0;
  // private Angle hoodSetpoint = Radians.of(0);

  public ShooterSim() {
    // var flywheelSystem = LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(3), 0.003, ShooterConstants.FLYWHEEL_GEAR_RATIO);
    // flywheelSim = new FlywheelSim(flywheelSystem, DCMotor.getNeoVortex(3));

    // var hoodSystem = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 0.02, 9);
    // hoodSim = new SingleJointedArmSim(hoodSystem, DCMotor.getNEO(1), gearing, armLengthMeters, minAngleRads, maxAngleRads, false, startingAngleRads);


  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {}

  @Override
  public void setVelocityFlywheel(double RPM) {

  }

  @Override
  public void setPositionHood(Angle angle) {

  }

  @Override
  public void stopHood() {

  }

  @Override
  public void stopFlywheel() {

  }
}
