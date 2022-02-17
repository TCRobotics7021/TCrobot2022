// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret;

public class autoturretaim extends CommandBase {
  double distancetotarget;

  double feedspeed;
  double shotspeed;
  double actualrpms;
  double getTA; 
  double targetX;
  double turret_speed;
  boolean finished; 
  double Pvalue;
  double Maxspeed; 
  double Minspeed; 
  /** Creates a new autoturretaim. */
  public autoturretaim() {
    addRequirements(RobotContainer.turret_subsystem);
    addRequirements(RobotContainer.limelight_subsystem);
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
    double targetX = RobotContainer.limelight_subsystem.getTx();
    
    
     
      

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
