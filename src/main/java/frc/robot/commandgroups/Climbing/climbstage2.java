// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Climbing;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Climbing.gantrycommand;
import frc.robot.commands.Climbing.liftcommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climbstage2 extends SequentialCommandGroup {
  /** Creates a new climbstage2. */
  public climbstage2() {
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
      new liftcommand(550)
     
);
  }
}
