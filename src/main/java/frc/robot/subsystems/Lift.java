
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



  public Lift() {
    SmartDashboard.putNumber("Test Height", 0);
   
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
    LiftMotor.set(ControlMode.PercentOutput, speed);
    holdposition = false;
  }

public void setposition(double target){
  targetposition = target; 
  holdposition = true;
}

public boolean atPosition(){
  if(currentposition > target-Constants.LIFT_TARGET_ACCURACY && currentposition < target+Constants.LIFT_TARGET_ACCURACY){
    return true;
    }
    else{
      return false;
    }
}


 public double Get_enc(){
//return 0;
return LiftMotor.getSelectedSensorPosition() * Constants.LIFT_ENC_CONV_FACTOR;

 }
  public void Set_enc(double Enc_set){
    LiftMotor.setSelectedSensorPosition(Enc_set);
  }

    @Override
  public void periodic() {

    if(holdposition){
      currentposition = RobotContainer.Lift_subsystem.Get_enc();
      difference = target - currentposition; 
      velocity = difference * Constants.LIFT_MOTOR_P;
      if(velocity > 0 && !top_limit.get()){
      RobotContainer.Lift_subsystem.setSpeed(0);
      }
      if(velocity < 0 && !bottom_limit.get()){
        RobotContainer.Lift_subsystem.setSpeed(0);
      }
    }


     if(LiftMotor.getSelectedSensorVelocity() < 0 && !bottom_limit.get()){
        setSpeed(0);
     }
     if(LiftMotor.getSelectedSensorVelocity() > 0 && !top_limit.get()){
       setSpeed(0);
     }

     if(bottom_limit.get() == false){
       Set_enc(Constants.LIFT_ENC_RESET_HEIGHT);
     }

    SmartDashboard.putNumber("Lift Encoder Position", LiftMotor.getSelectedSensorPosition() * Constants.LIFT_ENC_CONV_FACTOR);
    //This method will be called once per scheduler run
  }




}