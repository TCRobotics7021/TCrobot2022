// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class fwdeject extends CommandBase {
  /** Creates a new fwdeject. */
  double accuspeed;
  double inspeed;
  public fwdeject() {
    addRequirements(RobotContainer.intake_subsystem);
    addRequirements(RobotContainer.accumulator_subsystem);
    addRequirements(RobotContainer.shooter_subsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    inspeed = Constants.INTAKESPEED;
      accuspeed = Constants.ACCUSPEED;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter_subsystem.setShooterVelocity(500);
    if(RobotContainer.shooter_subsystem.atRPMS()){
    RobotContainer.shooter_subsystem.setfeedspeed(1);
    
    RobotContainer.accumulator_subsystem.setSpeed(accuspeed);
    RobotContainer.intake_subsystem.setSpeed(inspeed);
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
