// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class accumulator extends SubsystemBase {
  /** Creates a new accumulator. */
  public accumulator() {}
TalonFX accuMotor = new TalonFX(6);

DigitalInput accumulator_sensor = new DigitalInput(3);

  public void setSpeed(double accuspeed) {
    accuMotor.set(ControlMode.PercentOutput, -accuspeed);
  }

  public boolean isSensorBlocked(){
    return accumulator_sensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Accumulator Sensor", isSensorBlocked());
  }
}
