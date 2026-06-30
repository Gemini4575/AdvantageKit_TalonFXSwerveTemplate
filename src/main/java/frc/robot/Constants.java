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

  public static final class AdvancerConstants {

    public static final double ADVANCER_SPEED = -1.0;
    public static final double NEO_ADVANCER_SPEED = 1.0;
    public static final double ADVANCER_ROLLER_SPEED = -1.0;
    public static final int ADVANCER_MOTOR_ID = 1;
    public static final int ROLLER_MOTOR_ID = 9;
    public static final int NEO_ADVANCER_MOTOR_ID = 8; // was climber
    public static final boolean ADVANCER_MOTOR_INVERTED = true;
    public static final boolean NEO_ADVANCER_MOTOR_INVERTED = false;
    public static final boolean ROLLER_MOTOR_INVERTED = true;

    public static final double KRAKEN_ADCNAVER_SPEED = 1.0;

    public static final double ADVANCER_KA = 0.0;
    public static final double ADVANCER_KV = 0.0;
    public static final double ADVANCER_KS = 0.0;
  }

  public static final class NeoAdvancerConstants {
    public static final double KA = 0.0089868;
    public static final double KV = 0.00211;
    public static final double KS = 0.18438;
    public static final int NEO_ADVANCER_MOTOR_ID = 8;
    public static final boolean NEO_ADVANCER_MOTOR_INVERTED = false;
  }

  public static final class IntakeConstants {
    public static final int TOP_INTAKE_MOTOR_ID = 3;
    public static final int BOTTOM_INTAKE_MOTOR_ID = 4;
    public static final int INTAKE_ROTATOR_CAN_ID = 5;
    public static final double INTAKE_SPEED = 1.0;
    public static final double Intake_Hold_KP = 0.08;
    public static final double Intake_Down_SetPoint = 24.0;
    public static final double Intake_Up_SetPoint = 5;
  }

  public static final class ShooterConstants {
    // Constants
    public static final int SHOOTER_MOTOR_ID_0 = 13;
    public static final int SHOOTER_MOTOR_ID_1 = 14;
    public static final int SHOOTER_MOTOR_ID_2 = 15;
    public static final int SHOOTER_MOTOR_ID_3 = 16;
    public static final boolean SHOOTER_MOTOR_0_INVERTED = true;
    public static final boolean SHOOTER_MOTOR_1_INVERTED = true;
    public static final boolean SHOOTER_MOTOR_2_INVERTED = true;
    public static final boolean SHOOTER_MOTOR_3_INVERTED = true;
    public static final boolean SHOOTER_MOTOR_4_INVERTED = true;
    public static final double KA0 = 0.0077901;
    public static final double KA1 = 0.0089868;
    public static final double KA2 = 0.0085603;
    public static final double KA3 = 0.0090113;
    public static final double KV0 = 0.10839;
    public static final double KV1 = 0.10849;
    public static final double KV2 = 0.10867;
    public static final double KV3 = 0.1083;
    public static final double KS0 = 0.069689;
    public static final double KS1 = 0.087923;
    public static final double KS2 = 0.058578;
    public static final double KS3 = 0.037203;

    public static final double SHOOTER_KP = 0.00018;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
  }

  public static final class ShooterRPMConstants {
    public static final int HUB_SHOT = 2500;
    public static final int LADDER_SHOT = 4200;
    public static final int TRENCH_SHOT = 4000;
    public static final int ALLIANCE_WALL_SHOT = 5895;
  }

  public static class States {
    public static Boolean SHOOTER_ON = false;
    public static Boolean INTAKE_ON = false;
    public static Boolean CLIMBER_DOWN = false;
    public static Boolean ADVANCER_ON = false;
    public static Boolean INTAKE_IN = true;
    public static Boolean INTAKE_ON_AUTON = false;
  }
}
