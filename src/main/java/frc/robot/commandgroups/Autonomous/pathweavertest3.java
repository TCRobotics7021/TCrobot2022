// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class pathweavertest3 extends SequentialCommandGroup {
  /** Creates a new pathweavertest. */
  public pathweavertest3() {
   
    Trajectory trajectory1 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("test_path3");

    addCommands(
    new InstantCommand(()->{
      RobotContainer.drive_subsystem.resetOdometry(trajectory1.getInitialPose());
    }), 
    RobotContainer.drive_subsystem.createCommandForTrajectory(trajectory1, false).withTimeout(15).withName("debug")
    
    );
  }
}
