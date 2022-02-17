// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetHeading extends CommandBase {
  /** Creates a new ResetHeading. */
  public boolean finished = false;
  Rotation2d rotation = new Rotation2d(0,0);
  Pose2d pose = new Pose2d(0,0, rotation);
  public ResetHeading() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   RobotContainer.drive_subsystem.resetOdometry(pose);
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
