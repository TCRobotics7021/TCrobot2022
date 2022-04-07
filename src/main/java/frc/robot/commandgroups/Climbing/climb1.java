// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Climbing;

import java.util.Timer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Climbing.gantrycommand;
import frc.robot.commands.Climbing.liftcommand;
import frc.robot.commands.Climbing.waitforgantryathome;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climb1 extends SequentialCommandGroup {
  /** Creates a new climb1. */
  public climb1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
new gantrycommand(15),
//new waitforgantryathome(),
new liftcommand(-5),
//new WaitCommand(.5),
new gantrycommand(115),
new WaitCommand(.5),
new liftcommand(200),
new gantrycommand(518),
new liftcommand(655),
new gantrycommand(430),
new WaitCommand(2),
new liftcommand(400),
new gantrycommand(15),
//new waitforgantryathome(),
new liftcommand(0),
new WaitCommand(0),
new gantrycommand(104),
new liftcommand(5),
new WaitCommand(10)

    );
  }
}
