// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class CAN {
    // Dev drivebase CAN IDs
    public static final int FL_DRIVE_DEV = 3;
    public static final int FL_TURN_DEV = 9;
    public static final int FR_DRIVE_DEV = 5;
    public static final int FR_TURN_DEV = 6;
    public static final int BL_DRIVE_DEV = 4;
    public static final int BL_TURN_DEV = 7;
    public static final int BR_DRIVE_DEV = 2;
    public static final int BR_TURN_DEV = 8;

    // Comp drivebase CAN IDs
    public static final int FL_DRIVE_COMP = 3;
    public static final int FL_TURN_COMP = 7;
    public static final int FR_DRIVE_COMP = 4;
    public static final int FR_TURN_COMP = 8;
    public static final int BL_DRIVE_COMP = 2;
    public static final int BL_TURN_COMP = 6;
    public static final int BR_DRIVE_COMP = 9;
    public static final int BR_TURN_COMP = 5;

    // Augers
    public static final int AUGER = 10;
    // Intake
    public static final int INTAKE_RETRACTION = 11;
    public static final int INTAKE_FIXED_ROLLER = 12;
    public static final int INTAKE_EXTENDING_ROLLERS = 13;
    // Climb
    public static final int CLIMB = 14;
    // Shooter
    public static final int FLYWHEEL_LEADER = 15;
    public static final int FLYWHEEL_FOLLOWER_1 = 16;
    public static final int FLYWHEEL_FOLLOWER_2 = 17;
    public static final int VARIABLE_HOOD = 18;
    public static final int KICKER = 19;
  }
}
