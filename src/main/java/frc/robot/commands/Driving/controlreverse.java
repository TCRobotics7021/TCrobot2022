// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class controlreverse extends CommandBase {
  /** Creates a new controlreverse. */

  boolean finished = false;
  public controlreverse() {
    addRequirements(RobotContainer.drive_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  

  if(RobotContainer.drive_subsystem.controlreverse = true){
    RobotContainer.drive_subsystem.controlreverse = false;
  }
  else{
    RobotContainer.drive_subsystem.controlreverse = true; 
  }
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
