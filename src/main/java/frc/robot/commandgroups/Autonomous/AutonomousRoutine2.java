// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Climbing.MoveLiftandGantryHome;
import frc.robot.commands.Climbing.gantrycommand;
import frc.robot.commands.Driving.DriveCoast;
import frc.robot.commands.Other.intakecommand;
import frc.robot.commands.Shooting.AutoShoot2Ball;
import frc.robot.commands.Shooting.AutonomousShooting;
import frc.robot.commands.Shooting.autoturretaim;
import frc.robot.commands.Shooting.defaultshooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRoutine2 extends SequentialCommandGroup {
  /** Creates a new AutonomousRoutine2. */
  public AutonomousRoutine2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //once it gets to the end of the path the intake shuts off
    addCommands(   
      //new gantrycommand(0),
      new defaultshooter().withTimeout(.05),
      new ParallelCommandGroup( new DriveFirstPathAndIntake("AR2 Path1"),new MoveLiftandGantryHome()),
      //new MoveLiftandGantryHome()),
      //new DriveFirstPathAndIntake("AR2 Path1"),
      new AutoShoot2Ball(2150),
      new DrivePathAndIntake("AR2 Path2"),
     // new intakecommand().withTimeout(1),
      new DrivePathAndIntake("AR2 Path3"),
      new intakecommand().withTimeout(.25),
      new ParallelRaceGroup(new DrivePathAndIntake("AR2 Path4"), new autoturretaim()),
      new AutonomousShooting(2150).withTimeout(5.3),
      new DriveCoast()

    );
  }
}
