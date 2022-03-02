// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.limelight;

public class AutonomousShooting extends CommandBase {
  /** Creates a new AutonomousShooting. */
  double shooterspeed;
  double feedspeed;
  double Maxspeed;
  double Minspeed;
  double Pvalue;
  double turret_speed;
  double targetX;
  double actualrpms;
  boolean Startedshooting;

  public AutonomousShooting(double shooterspeed) {
    this.shooterspeed = shooterspeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter_subsystem);
    addRequirements(RobotContainer.limelight_subsystem);
    addRequirements(RobotContainer.turret_subsystem);
    addRequirements(RobotContainer.accumulator_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedspeed = SmartDashboard.getNumber("feedspeed", Constants.FEEDSPEED);
    Maxspeed = SmartDashboard.getNumber("Aim Max", Constants.MAX_AIM_SPEED);
    Minspeed = SmartDashboard.getNumber("Aim Min", Constants.MIN_AIM_SPEED);
    Pvalue = SmartDashboard.getNumber("Aim P", Constants.AIM_P);
    targetX = 2;
    RobotContainer.turret_subsystem.setcoastmode();
    Startedshooting = false;
  
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
        turret_speed = 0;
      }
  
  
      if (Startedshooting == false) {
          RobotContainer.turret_subsystem.setSpeed(turret_speed);
      }else {
         RobotContainer.turret_subsystem.setSpeed(0);
      }
  
        actualrpms = RobotContainer.shooter_subsystem.getshooterspeed();
        RobotContainer.shooter_subsystem.setShooterVelocity(shooterspeed);
      
      if (targetX <= 1 && targetX >= -1 && RobotContainer.shooter_subsystem.atRPMS()){
        if (!RobotContainer.shooter_subsystem.isSensorBlockedWithoffdelay()){
          RobotContainer.accumulator_subsystem.setSpeed(Constants.ACCUSPEED);
        }
        RobotContainer.shooter_subsystem.setfeedspeed(feedspeed);
        RobotContainer.turret_subsystem.setbrakemode();
        Startedshooting = true;
      }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter_subsystem.setfeedspeed(0);
    RobotContainer.accumulator_subsystem.setSpeed(0);
    RobotContainer.turret_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
