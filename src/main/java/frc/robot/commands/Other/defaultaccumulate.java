// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Other;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class defaultaccumulate extends CommandBase {
  /** Creates a new defaultaccumulate. */
  public defaultaccumulate() {
    // Use addRequirements() here to declare subsystem dependencies.

  addRequirements(RobotContainer.accumulator_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if (RobotContainer.accumulator_subsystem.isSensorBlocked() && !RobotContainer.shooter_subsystem.isSensorBlockedWithdelay()) {
      RobotContainer.accumulator_subsystem.setSpeed(Constants.ACCUSPEED);
    }
    else {
      RobotContainer.accumulator_subsystem.setSpeed(0);
    }
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
