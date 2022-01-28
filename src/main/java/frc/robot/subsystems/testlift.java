// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


public class testlift extends SubsystemBase {
  /** Creates a new testlift. */
  

  public double targetposition;
  //private WPI_TalonFX LiftMotor = new WPI_TalonFX(7);
  public CANSparkMax LiftMotor = new CANSparkMax(7, MotorType.kBrushless);
  public RelativeEncoder  LiftMotor_enc = LiftMotor.getEncoder();
  private DigitalInput top_limit = new DigitalInput(1);
  private DigitalInput bottom_limit = new DigitalInput(0);
  //private Solenoid latch = new Solenoid(11,0);
  



  public testlift() {
    SmartDashboard.putNumber("Test Height", 0);
   
  }
  public void setcoastmode(){
    //LiftMotor.setNeutralMode(NeutralMode.Coast);
  }
  public void setbrakemode(){
    //LiftMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setSpeed(double speed) {
    if(speed < 0 && !bottom_limit.get()){
      speed = 0;
    }
    if(speed > 0 && !top_limit.get()){
      speed = 0;
    }
    LiftMotor.set(speed);

  }

 public double Get_enc(){

return LiftMotor_enc.getPosition()* Constants.LIFT_ENC_CONV_FACTOR;

 }
  public void Set_enc(double Enc_set){
    LiftMotor_enc.setPosition(Enc_set);
  }

    @Override
  public void periodic() {
  

    if(LiftMotor_enc.getVelocity() < 0 && !bottom_limit.get()){
       setSpeed(0);
    }
    if(LiftMotor_enc.getVelocity() > 0 && !top_limit.get()){
      setSpeed(0);
    }

    if(bottom_limit.get() == false){
      Set_enc(Constants.LIFT_ENC_RESET_HEIGHT);
    }
    SmartDashboard.putNumber("Lift Encoder Position", LiftMotor_enc.getPosition() * Constants.LIFT_ENC_CONV_FACTOR);
    // This method will be called once per scheduler run
  }

}
