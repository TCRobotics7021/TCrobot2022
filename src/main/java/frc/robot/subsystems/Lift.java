
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Lift extends SubsystemBase {
  public double targetposition;
  private WPI_TalonFX LiftMotor = new WPI_TalonFX(11);
 
  private DigitalInput top_limit = new DigitalInput(1);
  private DigitalInput bottom_limit = new DigitalInput(0);
 
  double currentposition = 0;
  double difference = 0;
  double target = 0;
  double velocity = 0;
  boolean holdposition = false;
  public double MaxSpeed = 1;


  public Lift() {
    holdposition = false;
    targetposition = 0;
  }
  public void setcoastmode(){
    LiftMotor.setNeutralMode(NeutralMode.Coast);
  }
  public void setbrakemode(){
    LiftMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setSpeed(double speed) {
    if(speed < 0 && !bottom_limit.get()){
      speed = 0;
    }
    if(speed > 0 && !top_limit.get()){
      speed = 0;
    }
    LiftMotor.set(ControlMode.PercentOutput, -speed);
    setbrakemode();
    holdposition = false;
  }

  public void setmaxspeed(double Max){
    MaxSpeed = Max;
  }

  public void setposition(double target){
    targetposition = target; 
    holdposition = true;
  }

  public boolean atPosition(){
    if(currentposition > targetposition-Constants.LIFT_TARGET_ACCURACY && currentposition < targetposition+Constants.LIFT_TARGET_ACCURACY){
      return true;
      }
      else{
        return false;
      }
  }

  public double Get_enc(){
 
  return LiftMotor.getSelectedSensorPosition() * Constants.LIFT_ENC_CONV_FACTOR;

  }
  public void Set_enc(double Enc_set){
    LiftMotor.setSelectedSensorPosition(Enc_set);
  }

  public boolean atBottom(){
    return !bottom_limit.get();
  }


    @Override
  public void periodic() {

    if(holdposition){
      currentposition = Get_enc();
      difference = targetposition - currentposition; 
      velocity = difference * Constants.LIFT_MOTOR_P;
      if(Math.abs(velocity) < Constants.LIFT_MOTOR_MIN && !atPosition()){
        if(velocity>0){
          velocity = Constants.LIFT_MOTOR_MIN;
        }else{
          velocity = -Constants.LIFT_MOTOR_MIN;
        }
      }
      if(Math.abs(velocity) > MaxSpeed && !atPosition()){
        if(velocity>0){
          velocity = MaxSpeed;
        }else{
          velocity = -MaxSpeed;
        }
      }
    
      if(velocity > 0 && !top_limit.get()){
        velocity = 0;
      }
      else if(velocity < 0 && !bottom_limit.get()){
        velocity = 0;
      }

    
      
        LiftMotor.set(ControlMode.PercentOutput, -velocity);
        setbrakemode();
    }

     if(-LiftMotor.getSelectedSensorVelocity() < 0 && !bottom_limit.get()){
        LiftMotor.set(ControlMode.PercentOutput, 0);
     }
     if(-LiftMotor.getSelectedSensorVelocity() > 0 && !top_limit.get()){
        LiftMotor.set(ControlMode.PercentOutput, 0);
     }

     if(!bottom_limit.get()){
       Set_enc(Constants.LIFT_ENC_RESET_HEIGHT);
     }

     if(Constants.SHOW_DATA){
          SmartDashboard.putNumber("Lift Encoder Position", Get_enc());
          
          SmartDashboard.putBoolean("Lift Top Limit", !top_limit.get());
          SmartDashboard.putBoolean("Lift Bottom Limit", !bottom_limit.get());
     }
    //This method will be called once per scheduler run
  }




}