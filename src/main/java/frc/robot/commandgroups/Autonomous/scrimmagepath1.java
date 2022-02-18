// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Driving.drivebrake;
import frc.robot.commands.Shooting.Aim_and_shoot_turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scrimmagepath1 extends SequentialCommandGroup {
  /** Creates a new scrimmagepath1. */
  public scrimmagepath1() {

    addCommands(
 
    new DriveFirstPathAndIntake("Scrimmage1part1"),
    new Aim_and_shoot_turret().withTimeout(3),
    new DrivePathAndIntake("Scrimmage1part2"),
    new DrivePath("Scrimmage1part3"),
    new Aim_and_shoot_turret().withTimeout(3),
    new drivebrake()
    
    );
  }
}
