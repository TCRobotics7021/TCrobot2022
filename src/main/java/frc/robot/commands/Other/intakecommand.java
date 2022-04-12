// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Other;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Climbing.gantrycommand;

public class intakecommand extends CommandBase {
double accuspeed;
  double inspeed;
  /** Creates a new intake. */
  public intakecommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake_subsystem);
    addRequirements(RobotContainer.accumulator_subsystem);
    addRequirements(RobotContainer.gantry_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      inspeed = Constants.INTAKESPEED;
      accuspeed = Constants.ACCUSPEED;
      RobotContainer.gantry_subsystem.setbrakemode();
      RobotContainer.gantry_subsystem.setposition(Constants.GANTRY_EXTEND_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    
    
    
    
    if(RobotContainer.shooter_subsystem.isSensorBlocked() && RobotContainer.accumulator_subsystem.isSensorBlocked()){
      RobotContainer.accumulator_subsystem.setSpeed(0);
    }
    else{
    RobotContainer.accumulator_subsystem.setSpeed(accuspeed);
    }
    RobotContainer.intake_subsystem.setSpeed(inspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.accumulator_subsystem.setSpeed(0);
    RobotContainer.intake_subsystem.setSpeed(0);
    //RobotContainer.gantry_subsystem.Intimer.delay(.35);
    RobotContainer.gantry_subsystem.setposition(Constants.GANTRY_REST_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
