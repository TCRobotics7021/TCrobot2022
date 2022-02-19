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

public class shooter extends SubsystemBase {
TalonFX feedmotor = new TalonFX(7);
TalonFX feedmotor2 = new TalonFX(8);
  TalonFX shotmotor = new TalonFX(9);
  DigitalInput FeederSensor = new DigitalInput(3);

  Timer sensor_on_delay = new Timer();
  /** Creates a new shooter. */
  public shooter() {}

public double getshooterspeed(){
double shooterspeed;
shooterspeed = shotmotor.getSelectedSensorVelocity();
  return shooterspeed;

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

public void setfeedspeed(double feedspeed){
feedmotor2.set(ControlMode.PercentOutput, feedspeed);
  feedmotor.set(ControlMode.PercentOutput, -feedspeed);
}
  public void setshotSpeed(double shotspeed){

    shotmotor.set(ControlMode.PercentOutput, shotspeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Feeder Sensor", isSensorBlocked());

    if (isSensorBlocked() == true) {
      sensor_on_delay.start();
    }
     else {
      sensor_on_delay.reset();
    }
  }
}
