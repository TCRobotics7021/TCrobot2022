// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DefaultTurret extends CommandBase {
 
 double Maxspeed;
 double Minspeed;
 double Pvalue;
 double targetX;
 double turret_speed;
 boolean Startedshooting;

 
  /** Creates a new DefaultTurret. */
  public DefaultTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turret_subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    Maxspeed = SmartDashboard.getNumber("Aim Max", Constants.MAX_AIM_SPEED);
    Minspeed = SmartDashboard.getNumber("Aim Min", Constants.MIN_AIM_SPEED);
    Pvalue = SmartDashboard.getNumber("Aim P", Constants.AIM_P);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
if(RobotContainer.OPpanel.getRawButton(9) || RobotContainer.turret_subsystem.AUTOAIM == true) {
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
  
  
      
         RobotContainer.turret_subsystem.setSpeed(turret_speed);
      
} else {
  RobotContainer.turret_subsystem.setSpeed(0);
  RobotContainer.turret_subsystem.setcoastmode();
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
