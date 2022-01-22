// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class liftcommand extends CommandBase {
double velocity; 
double target;
double currentposition;
boolean finish;
  /** Creates a new liftcommand. */
  public liftcommand(double target) {

    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.lift_subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
currentposition = RobotContainer.lift_subsystem.Get_enc();
if(currentposition > target){
  RobotContainer.lift_subsystem.setSpeed(-.5);
}
if(currentposition < target){
  RobotContainer.lift_subsystem.setSpeed(.5);
}
if(currentposition > target-Constants.LIFT_TARGET_ACCURACY && currentposition < target+Constants.LIFT_TARGET_ACCURACY){
RobotContainer.lift_subsystem.setSpeed(0);
finish = true;
}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.lift_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
    
  }
}
