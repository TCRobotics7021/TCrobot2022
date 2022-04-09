// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climbing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ManualLift extends CommandBase {
  /** Creates a new Lift. */

double speed;

  public ManualLift(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.Lift_subsystem);
    addRequirements(RobotContainer.shooter_subsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.Lift_subsystem.setSpeed(speed);
    RobotContainer.shooter_subsystem.TurnOffIdle();
    RobotContainer.shooter_subsystem.setShooterVelocity(0);
    RobotContainer.Lift_subsystem.setmaxspeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.Lift_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
