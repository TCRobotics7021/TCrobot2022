// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commandgroups.Autonomous.AutonomousRoutine1;
import frc.robot.commandgroups.Autonomous.AutonomousRoutine2;
import frc.robot.commandgroups.Autonomous.AutonomousRoutine3;
import frc.robot.commandgroups.Autonomous.AutonomousRoutine4;
import frc.robot.commandgroups.Autonomous.AutonomousRoutine5;
import frc.robot.commandgroups.Climbing.climb1;
import frc.robot.commandgroups.Climbing.climbstage2;
import frc.robot.commands.TurretTurn;
import frc.robot.commands.Climbing.ManualGantry;
import frc.robot.commands.Climbing.ManualLift;
import frc.robot.commands.Climbing.MoveLiftandGantryHome;
import frc.robot.commands.Climbing.gantrycommand;
import frc.robot.commands.Climbing.liftcommand;
import frc.robot.commands.Driving.ArcadeDrive;
import frc.robot.commands.Driving.ConstantDrive;
import frc.robot.commands.Driving.controlreverse;
import frc.robot.commands.Driving.drivebrake;
import frc.robot.commands.Driving.turbo_drive;
import frc.robot.commands.Other.ResetHeading;
import frc.robot.commands.Other.Song;
import frc.robot.commands.Other.cancel;
import frc.robot.commands.Other.defaultaccumulate;
import frc.robot.commands.Other.defaultintake;
import frc.robot.commands.Other.ejector;
import frc.robot.commands.Other.intakecommand;
import frc.robot.commands.Shooting.Aim_and_shoot_turret;
import frc.robot.commands.Shooting.AutoShoot1Ball;
import frc.robot.commands.Shooting.AutoShoot2Ball;
import frc.robot.commands.Shooting.AutonomousShooting;
import frc.robot.commands.Shooting.DefaultTurret;
import frc.robot.commands.Shooting.aim_limelight;
import frc.robot.commands.Shooting.autoturretaim;
import frc.robot.commands.Shooting.defaultlimelight;
import frc.robot.commands.Shooting.defaultshooter;
import frc.robot.commands.Shooting.fwdeject;
import frc.robot.commands.Shooting.turretscan;
import frc.robot.subsystems.Gantry;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.accumulator;
import frc.robot.subsystems.drive;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

  public static Joystick LeftJoystick = new Joystick(1);
  public static Joystick RightJoystick = new Joystick(0);
  public static Joystick OPpanel = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    AutonomousChooser.setDefaultOption("AR1: One Ball and Shoot", new AutonomousRoutine1());
    AutonomousChooser.addOption("AR2: 4 Balls", new AutonomousRoutine2());
    AutonomousChooser.addOption("AR3: ", new AutonomousRoutine3());
    AutonomousChooser.addOption("AR4: ", new AutonomousRoutine4());
    AutonomousChooser.addOption("AR5: Do Nothing", new AutonomousRoutine5());


    SmartDashboard.putData("Auto Commands", AutonomousChooser);




    drive_subsystem.setDefaultCommand(new ArcadeDrive());
    intake_subsystem.setDefaultCommand(new defaultintake());
    accumulator_subsystem.setDefaultCommand(new defaultaccumulate());
    shooter_subsystem.setDefaultCommand(new defaultshooter());
    turret_subsystem.setDefaultCommand(new DefaultTurret());
    //limelight_subsystem.setDefaultCommand(new defaultlimelight());
    
    //turret_subsystem.setDefaultCommand(new autoturretaim());
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
new JoystickButton(RightJoystick, 1).whileHeld(new Aim_and_shoot_turret());
new JoystickButton(RightJoystick, 2).whileHeld(new turbo_drive());
// new JoystickButton(RightJoystick, 2).whileHeld(new ejector(), true);
new JoystickButton(LeftJoystick, 2).whileHeld(new fwdeject(), true);

new JoystickButton(OPpanel, 8).whileHeld(new TurretTurn(Constants.TURRET_TURN_SPEED));
new JoystickButton(OPpanel, 12).whileHeld(new TurretTurn(-Constants.TURRET_TURN_SPEED));

new JoystickButton(OPpanel, 7).whileHeld(new ManualLift(-Constants.LIFT_MOTOR_MANUAL_POWER) );
new JoystickButton(OPpanel, 6).whileHeld(new ManualLift(Constants.LIFT_MOTOR_MANUAL_POWER) );

new JoystickButton(OPpanel, 15).whileHeld(new ManualGantry(Constants.GANTRY_MOTOR_MANUAL_POWER) );
new JoystickButton(OPpanel, 16).whileHeld(new ManualGantry(-Constants.GANTRY_MOTOR_MANUAL_POWER) );

new JoystickButton(OPpanel, 1).whenPressed(new climb1(), false);
new JoystickButton(OPpanel, 2).whenPressed(new climbstage2(), false);

new JoystickButton(OPpanel, 3).whileHeld(new cancel(), false);

new JoystickButton(RightJoystick, 3).whileHeld(new AutonomousShooting(2000));

new JoystickButton(RightJoystick, 11).whenPressed(new AutoShoot1Ball(2000));
new JoystickButton(RightJoystick, 12).whenPressed(new AutoShoot2Ball(2000));

new JoystickButton(RightJoystick, 13).whileHeld(new ConstantDrive());

// new JoystickButton(OPpanel, 5).whenPressed(new liftcommand(580));
// new JoystickButton(OPpanel, 9).whenPressed(new liftcommand(100));
//new JoystickButton(OPpanel, 5).whenPressed(new gantrycommand(100));
//new JoystickButton(OPpanel, 9).whenPressed(new gantrycommand(300));
new JoystickButton(OPpanel, 5).whenPressed(new MoveLiftandGantryHome());

//new JoystickButton(RightJoystick, 10).whenPressed(new Song().withTimeout(10));

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
