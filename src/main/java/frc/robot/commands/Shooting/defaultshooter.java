// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class defaultshooter extends CommandBase {
  /** Creates a new defaultshooter. */
double shotspeed;


  public defaultshooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shotspeed = SmartDashboard.getNumber("shotspeed", Constants.SHOTSPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooter_subsystem.idle_on){
      RobotContainer.shooter_subsystem.setShooterVelocity(shotspeed);
    }else{
      RobotContainer.shooter_subsystem.setShooterVelocity(0);
    }
  
  if ((RobotContainer.accumulator_subsystem.isSensorBlocked() || RobotContainer.shooter_subsystem.isSensorBlocked()) && !RobotContainer.shooter_subsystem.isSensorBlockedWithdelay()) {
      RobotContainer.shooter_subsystem.setfeedspeed(1);
    }
    else {
  RobotContainer.shooter_subsystem.setfeedspeed(0);
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
