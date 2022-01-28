// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.limelight;

public class aim_limelight extends CommandBase {

  double targetX;
  double Rspeed;
  double Lspeed;
  boolean finished; 
  /** Creates a new aim_limelight. */
  public aim_limelight() {
    addRequirements(RobotContainer.limelight_subsystem);
    addRequirements(RobotContainer.drive_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetX = RobotContainer.limelight_subsystem.getTx();
    if (targetX > 0){
      Lspeed = Constants.AIM_P * targetX;
      Rspeed = -Constants.AIM_P * targetX; 
      
    }
    if (targetX < 0){
      Lspeed = -Constants.AIM_P * targetX;
      Rspeed = Constants.AIM_P * targetX; 
    }

    if (targetX <= 1 && targetX >= -1){
      finished = true; 
       
    }

    RobotContainer.drive_subsystem.setSpeed(Rspeed, Lspeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive_subsystem.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
