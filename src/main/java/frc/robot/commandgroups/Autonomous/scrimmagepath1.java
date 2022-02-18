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
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Trajectory scrimmagepath1part1 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("Scrimmage1part1");
    Trajectory scrimmagepath1part2 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("Scrimmage1part2");
    Trajectory scrimmagepath1part3 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("Scrimmage1part3");
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand(()->{
      RobotContainer.drive_subsystem.resetOdometry(scrimmagepath1part1.getInitialPose());
    }), 
    new DrivePathAndIntake(scrimmagepath1part1),
    new Aim_and_shoot_turret().withTimeout(5),
    new DrivePathAndIntake(scrimmagepath1part2),
    new drivepath(scrimmagepath1part3),
    new Aim_and_shoot_turret().withTimeout(4),
    new drivebrake()
    
    );
  }
}
