// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Other;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Song extends CommandBase {
  /** Creates a new Song. */

  Orchestra orchestra;
  String song1 = "song1.chrp";

  public Song() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake_subsystem);
    addRequirements(RobotContainer.accumulator_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
    _instruments.add(RobotContainer.accumulator_subsystem.accuMotor);
    _instruments.add(RobotContainer.intake_subsystem.InMotor);
    orchestra = new Orchestra(_instruments);
    orchestra.loadMusic(song1);
    orchestra.play();


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
    return false;
  }
}
