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
  double distancetotarget;

  double feedspeed;
  double shotspeed;
  double actualrpms;
  double Pvalue;
  double Maxspeed; 
  double Minspeed; 
  /** Creates a new aim_and_shoot. */
  public aim_and_shoot() {
    addRequirements(RobotContainer.limelight_subsystem);
    addRequirements(RobotContainer.drive_subsystem);
    addRequirements(RobotContainer.shooter_subsystem);
    //addRequirements(RobotContainer.accumulator_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedspeed = SmartDashboard.getNumber("feedspeed", 0);
    //shotspeed = SmartDashboard.getNumber("shotspeed", 0);
    Maxspeed = SmartDashboard.getNumber("Aim Max", Constants.MAX_AIM_SPEED);
    Minspeed = SmartDashboard.getNumber("Aim Min", Constants.MIN_AIM_SPEED);
    Pvalue = SmartDashboard.getNumber("Aim P", Constants.AIM_P);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetX = RobotContainer.limelight_subsystem.getTx();
    distancetotarget = RobotContainer.limelight_subsystem.getDistance();
    
    if (distancetotarget > 150){ 
      shotspeed = SmartDashboard.getNumber("Long Range Power", Constants.LONGRANGEPOWER);
    } 
    if (distancetotarget >= 150 && distancetotarget <= 110){
      shotspeed = SmartDashboard.getNumber("Mid Range Power", Constants.MIDRANGEPOWER);
    }
    if (distancetotarget <110){
      shotspeed = SmartDashboard.getNumber("Short Range Power", Constants.SHORTRANGEPOWER);
    }
    
      Lspeed = -Pvalue * targetX;
      Rspeed = Pvalue * targetX; 
     
    
   

    if (Math.abs(Lspeed) > Maxspeed){
      Lspeed = Maxspeed * Math.signum(Lspeed);
    }

    if (Math.abs(Rspeed) > Maxspeed){
      Rspeed = Maxspeed * Math.signum(Rspeed);
    }

    if (Math.abs(Lspeed) < Minspeed){
      Lspeed = Minspeed * Math.signum(Lspeed);
    }
    if (Math.abs(Rspeed) < Minspeed){
      Rspeed = Minspeed * Math.signum(Rspeed);
    }

    if (targetX <= 1 && targetX >= -1){
      //finished = true; 
      Rspeed = 0;
      Lspeed = 0;
       RobotContainer.drive_subsystem.drivebrake();
    }


    RobotContainer.drive_subsystem.setSpeed(Rspeed, Lspeed);

     actualrpms = RobotContainer.shooter_subsystem.getshooterspeed();
    RobotContainer.shooter_subsystem.setshotSpeed(shotspeed);
    SmartDashboard.putNumber("RPMs", actualrpms);
    if (actualrpms > 7000 && targetX <= 1 && targetX >= -1){
      RobotContainer.accumulator_subsystem.setSpeed(Constants.ACCUSPEED);

    RobotContainer.shooter_subsystem.setfeedspeed(feedspeed);

}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter_subsystem.setfeedspeed(0);
    RobotContainer.shooter_subsystem.setshotSpeed(0);
    RobotContainer.accumulator_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
