// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Driving.DriveCoast;
import frc.robot.commands.Other.intakecommand;
import frc.robot.commands.Shooting.AutonomousShooting;
import frc.robot.commands.Shooting.defaultshooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRoutine1 extends SequentialCommandGroup {
  /** Creates a new AutonomousRoutine1. */
  public AutonomousRoutine1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new defaultshooter().withTimeout(.05),
      new DriveFirstPathAndIntake("AR1 Path1"),
      new AutonomousShooting(2100)
    );
  }
}