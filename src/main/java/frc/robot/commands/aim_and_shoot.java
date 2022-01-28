// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.basic.BasicTabbedPaneUI.TabSelectionHandler;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class aim_and_shoot extends CommandBase {
  double targetX;
  double Rspeed;
  double Lspeed;
  
  double feedspeed;
  double shotspeed;
  double actualrpms;
  /** Creates a new aim_and_shoot. */
  public aim_and_shoot() {
    addRequirements(RobotContainer.limelight_subsystem);
    addRequirements(RobotContainer.drive_subsystem);
    addRequirements(RobotContainer.shooter_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedspeed = SmartDashboard.getNumber("feedspeed", 0);
    shotspeed = SmartDashboard.getNumber("shotspeed", 0);
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
      Lspeed = 0;
      Rspeed = 0;
      RobotContainer.drive_subsystem.drivebrake();
      
       
    }


    RobotContainer.drive_subsystem.setSpeed(Rspeed, Lspeed);

     actualrpms = RobotContainer.shooter_subsystem.getshooterspeed();
    RobotContainer.shooter_subsystem.setshotSpeed(shotspeed);

    if (actualrpms > 3000 && targetX <= 1 && targetX >= -1){

    RobotContainer.shooter_subsystem.setfeedspeed(feedspeed);

}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter_subsystem.setfeedspeed(0);
    RobotContainer.shooter_subsystem.setshotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
