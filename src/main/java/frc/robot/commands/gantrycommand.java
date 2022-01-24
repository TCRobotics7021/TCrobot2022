// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class gantrycommand extends CommandBase {

  double gantryvelocity; 
  double gantrytarget;
  double gantrycurrentposition;
  double difference;
  boolean finish; 


  /** Creat1es a new gantrycommand. */
  public gantrycommand(double target) {

    this.gantrytarget = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.gantry_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    difference = gantrytarget - gantrycurrentposition; 
    gantryvelocity = difference * Constants.LIFT_MOTOR_P;
    RobotContainer.Lift_subsystem.setSpeed(gantryvelocity);
if(gantrycurrentposition > gantrytarget-Constants.GANTRY_TARGET_ACCURACY && gantrycurrentposition < gantrytarget+Constants.GANTRY_TARGET_ACCURACY){
RobotContainer.gantry_subsystem.gantrysetSpeed(0);
finish = true;
}
  }






  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.gantry_subsystem.gantrysetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
