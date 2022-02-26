// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */

double LeftJoystick_Y;
double RightJoystick_X;
double RSpeed;
double LSpeed;
double FBmulti;
double LRmulti;


  public ArcadeDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.drive_subsystem.drivecoast();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    FBmulti = Constants.FBMULTI;
    LRmulti = Constants.LRMULTI;
    LeftJoystick_Y = RobotContainer.RightJoystick.getY();
    RightJoystick_X = RobotContainer.LeftJoystick.getX();

    RSpeed = (LeftJoystick_Y * FBmulti) + (RightJoystick_X * LRmulti);
    LSpeed = (LeftJoystick_Y * FBmulti) - (RightJoystick_X * LRmulti);

    if(RobotContainer.drive_subsystem.controlreverse = true){
      RobotContainer.drive_subsystem.setSpeed(-RSpeed, -LSpeed);
    }
    else{
      RobotContainer.drive_subsystem.setSpeed(RSpeed, LSpeed);
    }
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.drive_subsystem.setSpeed(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
