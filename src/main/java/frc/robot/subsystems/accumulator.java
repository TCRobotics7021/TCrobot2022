// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class accumulator extends SubsystemBase {
  /** Creates a new accumulator. */
  TalonFX accuMotor = new TalonFX(6);
  DigitalInput accumulator_sensor = new DigitalInput(2);

  Timer delaytimer = new Timer();
  double motoron = 0;

  public accumulator() {}


  public void setSpeed(double accuspeed) {
    motoron = accuspeed;
  }

  public boolean isSensorBlocked(){
    return !accumulator_sensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Constants.SHOW_DATA){
      SmartDashboard.putBoolean("Accumulator Sensor", isSensorBlocked());
    }
    delaytimer.start();
    if((motoron > 0 && (!isSensorBlocked() || !RobotContainer.shooter_subsystem.isSensorBlocked())) || motoron < 0) {
      delaytimer.reset();
      accuMotor.set(ControlMode.PercentOutput, motoron);
    }

    if((motoron == 0 && delaytimer.get() > Constants.ACCUMULATOR_DELAY) || (isSensorBlocked() && RobotContainer.shooter_subsystem.isSensorBlocked())) {
      accuMotor.set(ControlMode.PercentOutput, 0);
    }
  }
}
