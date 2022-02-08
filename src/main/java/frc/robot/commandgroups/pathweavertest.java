// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.shootercommand;
import frc.robot.subsystems.drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pathweavertest extends SequentialCommandGroup {
  /** Creates a new pathweavertest. */
  public pathweavertest(drive drive_subsystem) {
    Trajectory trajectory1 = drive_subsystem.loadTrajectoryFromFile("insertfilename");
    Trajectory trajectory2 = drive_subsystem.loadTrajectoryFromFile("insertfilename2");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand(()->{
      drive_subsystem.resetOdometry(trajectory1.getInitialPose());
    }), 
    drive_subsystem.createCommandForTrajectory(trajectory1, false).withTimeout(15).withName("debug"),
    new shootercommand().withName("debug part 2"),
    drive_subsystem.createCommandForTrajectory(trajectory2, false).withTimeout(15).withName("debug part 3")
    




    );
  }
}
