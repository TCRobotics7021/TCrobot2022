// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.gantrycommand;
import frc.robot.commands.liftcommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climb1 extends SequentialCommandGroup {
  /** Creates a new climb1. */
  public climb1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
new liftcommand(1000),
//drive move forward actually backward
new liftcommand(500),
new liftcommand(550),
new gantrycommand(500),
new liftcommand(1000),
new gantrycommand(0),
new liftcommand(500)



    );
  }
}