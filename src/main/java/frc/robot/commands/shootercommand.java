// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class shootercommand extends CommandBase {
  double feedspeed;
  double shotspeed;
  double actualrpms;
  /** Creates a new shootercommand. */
  public shootercommand() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.shooter_subsystem);
    
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
    actualrpms = RobotContainer.shooter_subsystem.getshooterspeed();
    RobotContainer.shooter_subsystem.setshotSpeed(shotspeed);

//if (actualrpms > 3000){

  RobotContainer.shooter_subsystem.setfeedspeed(feedspeed);

//}


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.shooter_subsystem.setfeedspeed(0);
    //RobotContainer.shooter_subsystem.setshotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
