// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  /** Creates a new intake. */
  public intake() {}
TalonFX InMotor = new TalonFX(5);

 public void setSpeed(double inspeed) {

InMotor.set(ControlMode.PercentOutput, inspeed);

 }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
