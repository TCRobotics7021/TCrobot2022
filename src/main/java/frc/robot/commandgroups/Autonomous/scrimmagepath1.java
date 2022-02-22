// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Driving.DriveCoast;
import frc.robot.commands.Driving.drivebrake;
import frc.robot.commands.Other.intakecommand;
import frc.robot.commands.Shooting.Aim_and_shoot_turret;
import frc.robot.commands.Shooting.AutonomousShooting;
import frc.robot.commands.Shooting.defaultshooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scrimmagepath1 extends SequentialCommandGroup {
  /** Creates a new scrimmagepath1. */
  public scrimmagepath1() {

    addCommands(
    new defaultshooter().withTimeout(.05),
    new DriveFirstPathAndIntake("Path1"),
    new AutonomousShooting(.42).withTimeout(3),
    new DrivePathAndIntake("Path2"),
    new intakecommand().withTimeout(2),
    new DrivePathAndIntake("Path3"),
    new AutonomousShooting(.42).withTimeout(1.5),
    new DriveCoast()
    
    );
  }
}
