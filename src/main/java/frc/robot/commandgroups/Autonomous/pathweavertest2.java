// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pathweavertest2 extends SequentialCommandGroup {
  /** Creates a new pathweavertest. */
  public pathweavertest2() {
    Trajectory trajectory1 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("position1");
    Trajectory trajectory2 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("position2");
    Trajectory trajectory3 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("position3");
    Trajectory trajectory4 = RobotContainer.drive_subsystem.loadTrajectoryFromFile("position4");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand(()->{
      RobotContainer.drive_subsystem.resetOdometry(trajectory1.getInitialPose());
    }), 
    RobotContainer.drive_subsystem.createCommandForTrajectory(trajectory1, false).withTimeout(15).withName("debugtra1"),
    RobotContainer.drive_subsystem.createCommandForTrajectory(trajectory2, false).withTimeout(15).withName("debugtra2"),
    RobotContainer.drive_subsystem.createCommandForTrajectory(trajectory3, false).withTimeout(15).withName("debugtra3"),
    RobotContainer.drive_subsystem.createCommandForTrajectory(trajectory4, false).withTimeout(15).withName("debugtra4")
    
    );
  }
}
