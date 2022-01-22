// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.deaultshooter;
import frc.robot.commands.defaultaccumulate;
import frc.robot.commands.defaultintake;
import frc.robot.commands.intakecommand;
import frc.robot.commands.shootercommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.accumulator;
import frc.robot.subsystems.drive;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static drive drive_subsystem = new drive();
  public static intake intake_subsystem = new intake();
public static accumulator accumulator_subsystem = new accumulator();
public static shooter shooter_subsystem = new shooter();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  public static Joystick LeftJoystick = new Joystick(0);
  public static Joystick RightJoystick = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive_subsystem.setDefaultCommand(new ArcadeDrive());
    intake_subsystem.setDefaultCommand(new defaultintake());
    accumulator_subsystem.setDefaultCommand(new defaultaccumulate());
    shooter_subsystem.setDefaultCommand(new deaultshooter());
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
new JoystickButton(RightJoystick, 1).whileHeld(new shootercommand(), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
