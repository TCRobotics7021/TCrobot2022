// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gantry extends SubsystemBase {



  private WPI_TalonFX GantryMotor = new WPI_TalonFX(12);
  /** Creates a new Gantry. */
  private DigitalInput FWDLimit = new DigitalInput(6);
  private DigitalInput BWDLimit = new DigitalInput(7);

  boolean holdposition = false;
  double targetposition = 0;
  double currentposition = 0;
  double difference = 0;
  double target = 0;
  double velocity = 0;

  public Gantry() {
    holdposition = false;
    targetposition = 0;
  }

  public void setcoastmode(){
    GantryMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setbrakemode(){
    GantryMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setSpeed(double speed) {
    if(speed > 0 && !FWDLimit.get()){
      speed = 0;
    }
    if(speed < 0 && !BWDLimit.get()){
      speed = 0;
    }
    GantryMotor.set(ControlMode.PercentOutput, speed);
    holdposition = false;
  }

public void setposition(double target){
  targetposition = target; 
  holdposition = true;
}

public boolean atPosition(){
  if(currentposition > targetposition-Constants.GANTRY_TARGET_ACCURACY && currentposition < targetposition+Constants.GANTRY_TARGET_ACCURACY){
    return true;
    }
    else{
      return false;
    }
}

  public double Get_enc(){

  return  GantryMotor.getSelectedSensorPosition() * Constants.GANTRY_ENC_CONV_FACTOR;

 }
  public void Set_enc(double Enc_set){
    GantryMotor.setSelectedSensorPosition(Enc_set);
  }

  public boolean atFwdProx(){
    return !FWDLimit.get();
  }

  @Override
  public void periodic() {


    
    if(holdposition){
      currentposition = Get_enc();
      difference = targetposition - currentposition; 
      velocity = -difference * Constants.GANTRY_MOTOR_P;
      if(Math.abs(velocity) < Constants.GANTRY_MOTOR_MIN && !atPosition()){
        if(velocity>0){
          velocity = Constants.GANTRY_MOTOR_MIN;
        }else{
          velocity = -Constants.GANTRY_MOTOR_MIN;
        }
      }
      if(velocity > 0 && !FWDLimit.get()){
        velocity = 0;
      }
      else if(velocity < 0 && !BWDLimit.get()){
        velocity = 0;
      }
      
      GantryMotor.set(ControlMode.PercentOutput, velocity);
          setbrakemode();
        
    }

     if( GantryMotor.getSelectedSensorVelocity() > 0 && !FWDLimit.get()){
      GantryMotor.set(ControlMode.PercentOutput, 0);
     }
     if( GantryMotor.getSelectedSensorVelocity() < 0 && !BWDLimit.get()){
      GantryMotor.set(ControlMode.PercentOutput, 0);
     }

     if(!FWDLimit.get()){
       Set_enc(Constants.GANTRY_ENC_RESET_HEIGHT);
     }

     
  

     if(Constants.SHOW_DATA){
          SmartDashboard.putNumber("Gantry Encoder Position", Get_enc());
          SmartDashboard.putBoolean("Gantry FWD Limit", !FWDLimit.get());
          SmartDashboard.putBoolean("Gantry BWD Limit", !BWDLimit.get());
          SmartDashboard.putNumber("Lift Velocity", velocity);
          SmartDashboard.putNumber("Difference", difference);
     }
    // This method will be called once per scheduler run
  }
}
