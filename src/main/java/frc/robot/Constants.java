// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3; // Values are placeholders currently
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 1;
    public static final int RIGHT_FOLLOWER_ID = 2;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 80;
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5; // Values are placeholders currently
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 80;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.2;
    public static final double ROLLER_EJECT_FAST_VALUE = 0.38;
  }

  public static final class ArmConstants {
    public static final int ARM_LEADER_ID = 9; // Values are placeholders currently
    public static final int ARM_MOTOR_CURRENT_LIMIT = 80;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_UP_VALUE = 0.7;
    public static final double ARM_DOWN_VALUE = 0.5;
  }

  public static final class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 9; // Values are placeholders currently
    public static final int CLIMB_MOTOR_CURRENT_LIMIT = 80;
    public static final double CLIMB_MOTOR_VOLTAGE_COMP = 10;
    public static final double CLIMB_VALUE = 0.7;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
