// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

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
    RobotContainer.limelight_subsystem.setLEDmode(3);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetX = RobotContainer.limelight_subsystem.getTx();
    



    if (targetX > 0){
      Lspeed = -Constants.AIM_P * targetX;
      Rspeed = Constants.AIM_P * targetX; 
      
    }
    if (targetX < 0){
      Lspeed = -Constants.AIM_P * targetX;
      Rspeed = Constants.AIM_P * targetX; 
    }

    if (Math.abs(Lspeed) > Constants.MAX_AIM_SPEED){
      Lspeed = Constants.MAX_AIM_SPEED * Math.signum(Lspeed);
    }

    if (Math.abs(Rspeed) > Constants.MAX_AIM_SPEED){
      Rspeed = Constants.MAX_AIM_SPEED * Math.signum(Rspeed);
    }

    if (Math.abs(Lspeed) < Constants.MIN_AIM_SPEED){
      Lspeed = Constants.MIN_AIM_SPEED * Math.signum(Lspeed);
    }
    if (Math.abs(Rspeed) < Constants.MIN_AIM_SPEED){
      Rspeed = Constants.MIN_AIM_SPEED * Math.signum(Rspeed);
    }

    if (targetX <= 1 && targetX >= -1){
      //finished = true; 
      Rspeed = 0;
      Lspeed = 0;
      RobotContainer.drive_subsystem.drivebrake();
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
