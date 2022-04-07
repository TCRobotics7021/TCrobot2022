// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climbing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MoveLiftandGantryHome extends CommandBase {
  /** Creates a new MoveLiftandGantryHome. */
  public MoveLiftandGantryHome() {
    addRequirements(RobotContainer.Lift_subsystem);
    addRequirements(RobotContainer.gantry_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.Lift_subsystem.setSpeed(-.2);
    RobotContainer.gantry_subsystem.setSpeed(Constants.GANTRY_EXTEND_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.Lift_subsystem.atBottom() && RobotContainer.gantry_subsystem.atFwdProx()){
      return true;
    }else{
      return false;
    }
    
  }
}
