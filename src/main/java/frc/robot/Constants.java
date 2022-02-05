// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static double LIFT_ENC_RESET_HEIGHT = 1000;

  public static double LIFT_ENC_CONV_FACTOR = 1;

  public static double LIFT_TARGET_ACCURACY = 2;

  public static double LIFT_MOTOR_P = .02;

  public static double GANTRY_ENC_RESET_HEIGHT = 1000;

  public static double GANTRY_ENC_CONV_FACTOR = 1;

  public static double GANTRY_TARGET_ACCURACY = 2;

  public static double GANTRY_MOTOR_P = .02;


  public static double TURBO_FBMULTI = 1;

  public static double TURBO_LRMULTI = 1; 


  public static double AIM_P = .01; 

  public static double MAX_AIM_SPEED = .3;
  public static double MIN_AIM_SPEED = .1;

  public static double ACCUSPEED = .5;

}
