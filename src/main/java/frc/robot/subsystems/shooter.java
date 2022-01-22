// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
TalonFX feedmotor = new TalonFX(7);
  TalonFX shotmotor = new TalonFX(8);
  /** Creates a new shooter. */
  public shooter() {}

public double getshooterspeed(){
double shooterspeed;
shooterspeed = shotmotor.getSelectedSensorVelocity();
  return shooterspeed;

}

public void setfeedspeed(double feedspeed){

  feedmotor.set(ControlMode.PercentOutput, feedspeed);
}
  public void setshotSpeed(double shotspeed){

    shotmotor.set(ControlMode.PercentOutput, shotspeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
