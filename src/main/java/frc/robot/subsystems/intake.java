// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class intake extends SubsystemBase {
  /** Creates a new intake. */
  Timer delaytimer = new Timer();
  double motoron = 0 ;
  public intake() {}
TalonFX InMotor = new TalonFX(5);

 public void setSpeed(double inspeed) {
   motoron = inspeed; 



 }



  @Override
  public void periodic() {
    delaytimer.start();
    if((motoron > 0 && (!RobotContainer.accumulator_subsystem.isSensorBlocked() || !RobotContainer.shooter_subsystem.isSensorBlocked())) || motoron < 0) {
      delaytimer.reset();
      InMotor.set(ControlMode.PercentOutput, -motoron);
    }

    if((motoron == 0 && delaytimer.get() > Constants.INTAKE_DELAY) || (RobotContainer.accumulator_subsystem.isSensorBlocked() && RobotContainer.shooter_subsystem.isSensorBlocked())) {
      InMotor.set(ControlMode.PercentOutput, 0);
    }

    // This method will be called once per scheduler run
  }
}
