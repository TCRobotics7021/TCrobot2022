// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commandgroups.Autonomous.pathweavertest;
import frc.robot.commandgroups.Autonomous.pathweavertest2;
import frc.robot.commandgroups.Autonomous.pathweavertest3;
import frc.robot.commandgroups.Climbing.climb1;
import frc.robot.commands.Climbing.defaultliftcommand;
import frc.robot.commands.Driving.ArcadeDrive;
import frc.robot.commands.Driving.controlreverse;
import frc.robot.commands.Driving.drivebrake;
import frc.robot.commands.Driving.turbo_drive;
import frc.robot.commands.Other.ResetHeading;
import frc.robot.commands.Other.cancel;
import frc.robot.commands.Other.defaultaccumulate;
import frc.robot.commands.Other.defaultintake;
import frc.robot.commands.Other.intakecommand;
import frc.robot.commands.Shooting.aim_and_shoot;
import frc.robot.commands.Shooting.aim_limelight;
import frc.robot.commands.Shooting.autoturretaim;
import frc.robot.commands.Shooting.defaultshooter;
import frc.robot.commands.Shooting.turretscan;
import frc.robot.subsystems.Gantry;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.accumulator;
import frc.robot.subsystems.drive;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.testlift;
import frc.robot.subsystems.turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static drive drive_subsystem = new drive();
  public static intake intake_subsystem = new intake();
  public static accumulator accumulator_subsystem = new accumulator();
  public static shooter shooter_subsystem = new shooter();
  public static Lift Lift_subsystem = new Lift();
  public static Gantry gantry_subsystem = new Gantry();
  public static limelight limelight_subsystem = new limelight(); 
  public static turret turret_subsystem = new turret(); 

  SendableChooser AutonomousChooser = new SendableChooser<Command>();

  public static Joystick LeftJoystick = new Joystick(0);
  public static Joystick RightJoystick = new Joystick(1);
  public static Joystick OPpanel = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AutonomousChooser.setDefaultOption("Pathweavertest", new pathweavertest());
    AutonomousChooser.addOption("Pathweavertest2", new pathweavertest2());
    AutonomousChooser.addOption("Pathweavertest3", new pathweavertest3());

    SmartDashboard.putData("Auto Commands", AutonomousChooser);




    drive_subsystem.setDefaultCommand(new ArcadeDrive());
    intake_subsystem.setDefaultCommand(new defaultintake());
    accumulator_subsystem.setDefaultCommand(new defaultaccumulate());
    shooter_subsystem.setDefaultCommand(new defaultshooter());
   // turret_subsystem.setDefaultCommand(new autoturretaim());
    //Lift_subsystem.setDefaultCommand(new defaultliftcommand());


  

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


new JoystickButton(LeftJoystick, 1).whileHeld(new intakecommand(), true);
//new JoystickButton(RightJoystick, 1).whenPressed(new ResetHeading(), false);
//new JoystickButton(RightJoystick, 1).whileHeld(new shootercommand(), true);
new JoystickButton(OPpanel, 1).whenPressed(new climb1(), true);
new JoystickButton(OPpanel, 3).whileHeld(new cancel(), false);
new JoystickButton(LeftJoystick, 2).whileHeld(new drivebrake(), true);
new JoystickButton(RightJoystick, 2).whileHeld(new turbo_drive(), true);
//new JoystickButton(RightJoystick, 3).whileHeld(new aim_limelight(), true); 
new JoystickButton(RightJoystick, 4).whileHeld(new aim_and_shoot(), true);
new JoystickButton(OPpanel, 8).whenPressed(new controlreverse() , true);
//new JoystickButton(OPpanel, 4).whenPressed(new turretscan(), true); 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (Command) AutonomousChooser.getSelected();
  }
}
