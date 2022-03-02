// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
TalonFX feedmotor = new TalonFX(7);
TalonFX feedmotor2 = new TalonFX(8);
  TalonFX shotmotor = new TalonFX(9);
  DigitalInput FeederSensor = new DigitalInput(3);

double targetRPM;
public boolean idle_on = true;

  Timer sensor_on_delay = new Timer();
  Timer sensor_off_delay = new Timer();
  /** Creates a new shooter. */
  public shooter() {
    setupShooterMotor();
    setupShooterPID();
  }

public void TurnOnIdle(){
  idle_on = true;
}
public void TurnOffIdle(){
  idle_on = false;
}

public double getshooterspeed(){
double shooterspeed;
 shooterspeed = shotmotor.getSelectedSensorVelocity() / 2048 * 600;
  return shooterspeed;

}

private void setupShooterMotor() {
  shotmotor.setNeutralMode(NeutralMode.Coast);
}

private void setupShooterPID() {
  shotmotor.config_kP(0, SmartDashboard.getNumber("Shooter kP", Constants.SHOOTER_kP));
  shotmotor.config_kD(0, SmartDashboard.getNumber("Shooter kD", Constants.SHOOTER_kD));
  shotmotor.config_kF(0, SmartDashboard.getNumber("Shooter kF", Constants.SHOOTER_kF));
}

public boolean isSensorBlocked() {
  return !FeederSensor.get();
}

public boolean isSensorBlockedWithdelay() {
  if(sensor_on_delay.get() > SmartDashboard.getNumber("Shooter Sensor Delay Time", Constants.SHOOTERSENSORDELAYTIME)){
    return true;
  }
  else{
    return false;
  }
}
public boolean isSensorBlockedWithoffdelay() {
  if(sensor_off_delay.get() < SmartDashboard.getNumber("Shooter Sensor Off Delay Time", Constants.SHOOTERSENSOROFFDELAYTIME)){
    return true;
  }
  else{
    return false;
  }
}

  public void setfeedspeed(double feedspeed){
    feedmotor2.set(ControlMode.PercentOutput, feedspeed);
    feedmotor.set(ControlMode.PercentOutput, -feedspeed);
  }
  public void setshotSpeed(double shotspeed){
    shotmotor.set(ControlMode.PercentOutput, shotspeed);
  }

  public void setShooterVelocity(double velocity){
    shotmotor.set(ControlMode.Velocity, velocity * 2048 / 600);
    targetRPM = velocity;
  }

  public boolean atRPMS() {
    if (targetRPM * .9 < getshooterspeed() && targetRPM * 1.1 > getshooterspeed()) {
      return true;
    } else {
      return false;
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    if (isSensorBlocked() == true) {
      sensor_on_delay.start();
    }
     else {
      sensor_on_delay.reset();
    }
    if (isSensorBlocked() == false) {
      sensor_off_delay.start();
    }
     else {
      sensor_off_delay.reset();
    }

    if(Constants.SHOW_DATA){
      setupShooterPID();
      SmartDashboard.putBoolean("Feeder Sensor", isSensorBlocked());
      SmartDashboard.putBoolean("At RPMs", atRPMS());
      SmartDashboard.putNumber("Current RPMs", getshooterspeed());
    }
  }
}
