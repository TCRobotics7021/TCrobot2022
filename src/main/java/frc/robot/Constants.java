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

// Motor Ports
public static final int kLeftMotorPort1 = 4;
public static final int kLeftMotorPort2 = 2;

public static final int kRightMotorPort1 = 3;
public static final int kRightMotorPort2 = 1;

// results from characterization tool
public static final double ks = 0.696;
public static final double kv = 1.6; // was at 1.6
public static final double ka = 0.005; // was at .005
public static final double kp = 0.05; // was at .05

// Sets up gear ratio and other info
public static final double gearRatio = .1528;
public static final double wheelDiameterInches = 6;
public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
public static final double encoderTicksPerRev = 2048;

// Randoms
public static final double leftScaleConstant = 1.00;
public static final double rightScaleConstant = 1.00;




}
