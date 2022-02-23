// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Aim_and_shoot_turret extends CommandBase {
  double targetX;
  double turret_speed; 
  double distancetotarget;

  double feedspeed;
  double shotspeed;
  double actualrpms;
  double Pvalue;
  double Maxspeed; 
  double Minspeed; 
  /** Creates a new Aim_and_shoot_turret. */
  public Aim_and_shoot_turret() {
    addRequirements(RobotContainer.limelight_subsystem);
    addRequirements(RobotContainer.drive_subsystem);
    addRequirements(RobotContainer.shooter_subsystem);
    addRequirements(RobotContainer.turret_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedspeed = SmartDashboard.getNumber("feedspeed", 0);
    shotspeed = SmartDashboard.getNumber("shotspeed", 0);
    Maxspeed = SmartDashboard.getNumber("Aim Max", Constants.MAX_AIM_SPEED);
    Minspeed = SmartDashboard.getNumber("Aim Min", Constants.MIN_AIM_SPEED);
    Pvalue = SmartDashboard.getNumber("Aim P", Constants.AIM_P);

    distancetotarget = RobotContainer.limelight_subsystem.getDistance();
    
    // if (distancetotarget > 150){ 
    //   shotspeed = SmartDashboard.getNumber("Long Range Power", Constants.LONGRANGEPOWER);
    // } 
    // if (distancetotarget >= 150 && distancetotarget <= 110){
    //   shotspeed = SmartDashboard.getNumber("Mid Range Power", Constants.MIDRANGEPOWER);
    // }
    // if (distancetotarget <110){
    //   shotspeed = SmartDashboard.getNumber("Short Range Power", Constants.SHORTRANGEPOWER);
    // }
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive_subsystem.setSpeed(0, 0);
    RobotContainer.drive_subsystem.drivebrake();
    targetX = RobotContainer.limelight_subsystem.getTx();
    
    
     
      turret_speed = Pvalue * targetX; 
      if (Math.abs(turret_speed) > Maxspeed){
        turret_speed = Maxspeed * Math.signum(turret_speed);
      }
  
      
  
      if (Math.abs(turret_speed) < Minspeed){
        turret_speed = Minspeed * Math.signum(turret_speed);
      }
     
  
      if (targetX <= 1 && targetX >= -1){
        //finished = true; 
        turret_speed = 0;
         RobotContainer.turret_subsystem.setbrakemode();
         
      }
  
  
      RobotContainer.turret_subsystem.setSpeed(turret_speed);
  
       actualrpms = RobotContainer.shooter_subsystem.getshooterspeed();
      RobotContainer.shooter_subsystem.setShooterVelocity(shotspeed);
      

      if (RobotContainer.shooter_subsystem.atRPMS() && targetX <= 1 && targetX >= -1){
        if (!RobotContainer.shooter_subsystem.isSensorBlockedWithoffdelay()){
          RobotContainer.accumulator_subsystem.setSpeed(Constants.ACCUSPEED);
        }
  
      RobotContainer.shooter_subsystem.setfeedspeed(feedspeed);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter_subsystem.setfeedspeed(0);
    RobotContainer.shooter_subsystem.setShooterVelocity(0);
    RobotContainer.accumulator_subsystem.setSpeed(0);
    RobotContainer.turret_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
