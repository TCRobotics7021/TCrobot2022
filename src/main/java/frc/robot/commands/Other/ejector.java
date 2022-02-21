// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Other;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter;

public class ejector extends CommandBase {
  /** Creates a new ejector. */
  public ejector() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.accumulator_subsystem);
    addRequirements(RobotContainer.intake_subsystem);
    addRequirements(RobotContainer.shooter_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake_subsystem.setSpeed(-Constants.INTAKESPEED);
    RobotContainer.accumulator_subsystem.setSpeed(-Constants.ACCUSPEED);
    RobotContainer.shooter_subsystem.setfeedspeed(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
