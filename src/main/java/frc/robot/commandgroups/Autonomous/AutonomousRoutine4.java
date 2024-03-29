// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurretTurn;
import frc.robot.commands.Climbing.MoveLiftandGantryHome;
import frc.robot.commands.Climbing.gantrycommand;
import frc.robot.commands.Other.intakecommand;
import frc.robot.commands.Shooting.AutoShoot1Ball;
import frc.robot.commands.Shooting.AutoShoot2Ball;
import frc.robot.commands.Shooting.AutonomousShooting;
import frc.robot.commands.Shooting.autoturretaim;
import frc.robot.commands.Shooting.defaultshooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousRoutine4 extends SequentialCommandGroup {
  /** Creates a new AutonomousRoutine4. */
  public AutonomousRoutine4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new defaultshooter().withTimeout(.05),
      new ParallelCommandGroup( new DriveFirstPathAndIntake("AR4 Path1"),new MoveLiftandGantryHome()),
      new AutoShoot2Ball(2000),
      new ParallelCommandGroup( new DrivePathAndIntake("AR4 Path2"), new TurretTurn(-.3).withTimeout(.2)),
      new AutoShoot2Ball(2100),
      new DrivePathAndIntake("AR4 Path3"),
     new ParallelRaceGroup(new DrivePathAndIntake("AR4 Path4"), new autoturretaim()),
      new AutonomousShooting(2100).withTimeout(7.8)
    );
  }
}
