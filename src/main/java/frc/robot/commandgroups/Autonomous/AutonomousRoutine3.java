// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooting.AutonomousShooting;
import frc.robot.commands.Shooting.defaultshooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRoutine3 extends SequentialCommandGroup {
  /** Creates a new AutonomousRoutine3. */
  public AutonomousRoutine3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new defaultshooter().withTimeout(.05),
      new DriveFirstPathAndIntake("AR3 Path1"),  
      new AutonomousShooting(2000),
      new DrivePathAndIntake("AR3 Path2"),
      new drivepath("AR3 Path3"),
      new AutonomousShooting(2200)
    );
  }
}
