// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveFirstPath extends SequentialCommandGroup {
  /** Creates a new DriveFirstPath. */
  public DriveFirstPath(String pathname) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Trajectory path = RobotContainer.drive_subsystem.loadTrajectoryFromFile(pathname);
    
    addCommands(
      new InstantCommand(()->{RobotContainer.drive_subsystem.resetOdometry(path.getInitialPose());}),
      RobotContainer.drive_subsystem.createCommandForTrajectory(path, false) 
    );
  }
}
