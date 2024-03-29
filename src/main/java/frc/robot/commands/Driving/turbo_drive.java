// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class turbo_drive extends CommandBase {


double LeftJoystick_Y;
double RightJoystick_X;
double RSpeed;
double LSpeed;
double FBmulti;
double LRmulti;

  /** Creates a new turbo_drive. */
  public turbo_drive() {
    addRequirements(RobotContainer.drive_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    FBmulti = Constants.TURBO_FBMULTI;
    LRmulti = Constants.TURBO_LRMULTI;
    LeftJoystick_Y = RobotContainer.LeftJoystick.getY();
    RightJoystick_X = RobotContainer.RightJoystick.getX();

    RSpeed = (LeftJoystick_Y * FBmulti) + (RightJoystick_X * LRmulti);
    LSpeed = (LeftJoystick_Y * FBmulti) - (RightJoystick_X * LRmulti);

    RobotContainer.drive_subsystem.setSpeed(RSpeed, LSpeed);
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
