// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climbing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class liftcommand extends CommandBase {
double velocity; 
double target;
double currentposition;
double difference; 
boolean finish;
  /** Creates a new liftcommand. */
  public liftcommand(double target) {

    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.Lift_subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
RobotContainer.Lift_subsystem.targetposition = target;
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
currentposition = RobotContainer.Lift_subsystem.Get_enc();
difference = target - currentposition; 
velocity = difference * Constants.LIFT_MOTOR_P;
RobotContainer.Lift_subsystem.setSpeed(velocity);
if(currentposition > target-Constants.LIFT_TARGET_ACCURACY && currentposition < target+Constants.LIFT_TARGET_ACCURACY){
RobotContainer.Lift_subsystem.setSpeed(0);
finish = true;
}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.Lift_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
    
  }
}
