// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Gantry;

public class MoveGantryToLimit extends CommandBase {
  /** Creates a new MoveGantryToLimit. */
  public MoveGantryToLimit() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.gantry_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.gantry_subsystem.setSpeed(-.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.gantry_subsystem.atBwdProx()){
      return true;
    }else{
      return false;
    }
    
  }
}
