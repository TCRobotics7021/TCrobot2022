// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drive extends SubsystemBase {
  /** Creates a new drive. */

TalonFX FLmotor = new TalonFX(4);
TalonFX FRmotor = new TalonFX(3);
TalonFX BLmotor = new TalonFX(2);
TalonFX BRmotor = new TalonFX(1);

  public drive() {}

public void setSpeed(double rightspeed, double leftspeed){

  FRmotor.set(ControlMode.PercentOutput, -rightspeed);
  FLmotor.set(ControlMode.PercentOutput, leftspeed);
  BRmotor.set(ControlMode.PercentOutput, -rightspeed);
  BLmotor.set(ControlMode.PercentOutput, leftspeed);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
