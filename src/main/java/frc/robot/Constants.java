// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


// results from characterization tool
public static final double ks = 0.60069;
public static final double kv = 2.3549; // was at 1.6
public static final double ka = 0.3997; // was at .005
public static final double kp = .077925; // was at .05
public static final double track_width = 0.6431; // Has to be in meters


// Sets up gear ratio and other info
public static final double gearRatio = 84/8;
public static final double wheelDiameterInches = 6;
public static final double wheelCircumference = 0.4592;
public static final double encoderTicksPerRev = 2048;
public static final double metersPerEncoderTick = wheelCircumference / (encoderTicksPerRev * gearRatio);

// Randoms
public static final double leftScaleConstant = 1.00;
public static final double rightScaleConstant = 1.00;

  //Lift (lifts the robot)
  public static double LIFT_ENC_RESET_HEIGHT = 1000;
  public static double LIFT_ENC_CONV_FACTOR = 1;
  public static double LIFT_TARGET_ACCURACY = 2;
  public static double LIFT_MOTOR_P = .02;
  
  //gantry (moving lift)
  public static double GANTRY_ENC_RESET_HEIGHT = 1000;
  public static double GANTRY_ENC_CONV_FACTOR = 1;
  public static double GANTRY_TARGET_ACCURACY = 2;
  public static double GANTRY_MOTOR_P = .02;

  //turret limits
  public static double TURRET_ENC_CONV_FACTOR = 1; 

  //arcade drive
  public static double TURBO_FBMULTI = 1;
  public static double TURBO_LRMULTI = 1; 
  public static double FBMULTI = .5;
  public static double LRMULTI = .3;

  //turret
  public static double AIM_P = .01; 
  public static double MAX_AIM_SPEED = .3;
  public static double MIN_AIM_SPEED = .1;
  public static double TURRET_TURN_SPEED = .2;

  //accumulater 
  public static double ACCUSPEED = .5;
  public static double SHOOTERSENSORDELAYTIME = .07;
  public static double SHOOTERSENSOROFFDELAYTIME = .45;

  // Distance calculation 
  public static double DIST_CALC_A = 0.324;
  public static double DIST_CALC_B = -2.44;
  public static double DIST_CALC_C = 96.4;

  //ditsance shooting 
  public static double SHORTRANGEPOWER = 2000;
  public static double MIDRANGEPOWER = 2100;
  public static double LONGRANGEPOWER = 2500;

  public static double INTAKESPEED = 1;
  public static double FEEDSPEED = 1;
  public static double SHOTSPEED = 2100;

  //Shooter PID
  public static double SHOOTER_kP = .25;
  public static double SHOOTER_kD = 0;
  public static double SHOOTER_kF = .044;
}
