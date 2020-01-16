/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DrivetrainConstants {
    public static final int LEFT_MASTER_CAN_ID = 14;
    public static final int RIGHT_MASTER_CAN_ID = 16;

    public static final int LEFT_SLAVE1_CAN_ID = 10;
    public static final int LEFT_SLAVE2_CAN_ID = 11;
    public static final int RIGHT_SLAVE1_CAN_ID = 12;
    public static final int RIGHT_SLAVE2_CAN_ID = 13;

    public static final int LEFT_ENCODER_A = 6;
    public static final int LEFT_ENCODER_B = 7;
    public static final int RIGHT_ENCODER_A = 9;
    public static final int RIGHT_ENCODER_B = 8;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0d);

    public static final double PULSES_PER_REVOLUTION = 256;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / PULSES_PER_REVOLUTION;
  }

  public static final class ControlConstants {
    public static final int DRIVER_ID = 0;
  }
}
