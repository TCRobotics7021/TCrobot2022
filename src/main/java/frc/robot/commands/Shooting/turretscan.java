// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.limelight;

public class turretscan extends CommandBase {
  double targetarea = 0; 
  boolean finished = false; 
  
  /** Creates a new turretscan. */
  public turretscan() {
    addRequirements(RobotContainer.limelight_subsystem);
    addRequirements(RobotContainer.turret_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight_subsystem.setLEDmode(3);
    finished = false; 
    RobotContainer.turret_subsystem.setSpeed(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.turret_subsystem.isLeftLim() == true){
     
      RobotContainer.turret_subsystem.setSpeed(-.5);

    }
    if(RobotContainer.turret_subsystem.isRightLim() == true){
     
      RobotContainer.turret_subsystem.setSpeed(.5);

    }
    targetarea = RobotContainer.limelight_subsystem.getTa();
    if(targetarea >= 1){
      finished = true; 
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
