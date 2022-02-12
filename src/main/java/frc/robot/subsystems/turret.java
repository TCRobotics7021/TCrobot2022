// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class turret extends SubsystemBase {
  
  public final TalonFX TurretMotor = new TalonFX(13);
  private static final String Enc_set = null;
  private DigitalInput LLimit = new DigitalInput(4);
  private DigitalInput RLimit = new DigitalInput(5);
  /** Creates a new turret. */
  public turret() {}

  public void setcoastmode(){
    TurretMotor.setNeutralMode(NeutralMode.Coast);
  }
  public void setbrakemode(){
    TurretMotor.setNeutralMode(NeutralMode.Brake);
  }
  public void setSpeed(double speed) {
    if(speed < 0 && !LLimit.get()){
      speed = 0;
    }
    if(speed > 0 && !RLimit.get()){
      speed = 0;
    }
    TurretMotor.set(ControlMode.PercentOutput,speed);

  }

 public double Get_enc(){

return TurretMotor.getSelectedSensorPosition() * Constants.TURRET_ENC_CONV_FACTOR;
 }

  public void Set_enc(double ENC_set){
    //TurretMotor.setSelectedSensorPosition(Enc_set);
    TurretMotor.setSelectedSensorPosition(ENC_set);
  } 
  public boolean isLeftLim(){
    return !LLimit.get();
  }
  public boolean isRightLim(){
    return !RLimit.get();
  }

  @Override
  public void periodic() {
    if(TurretMotor.getSelectedSensorVelocity() < 0 && !LLimit.get()){
        setSpeed(0);
       }
      if(TurretMotor.getSelectedSensorVelocity() > 0 && !RLimit.get()){
         setSpeed(0);
       }
  
       if(LLimit.get() == false){
         Set_enc(Constants.LIFT_ENC_RESET_HEIGHT);
       }
      SmartDashboard.putNumber("Turret Encoder Position", TurretMotor.getSelectedSensorPosition() * Constants.TURRET_ENC_CONV_FACTOR);
    // This method will be called once per scheduler run
  }
}
