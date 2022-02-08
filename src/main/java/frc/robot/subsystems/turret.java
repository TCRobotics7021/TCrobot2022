// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class turret extends SubsystemBase {
  

  private static final String Enc_set = null;
  private DigitalInput LLimit = new DigitalInput(9);
  private DigitalInput RLimit = new DigitalInput(11);
  /** Creates a new turret. */
  public turret() {}

  public void setcoastmode(){
    //GantryMotor.setNeutralMode(NeutralMode.Coast);
  }
  public void setbrakemode(){
    //GantryMotor.setNeutralMode(NeutralMode.Brake);
  }
  public void setSpeed(double speed) {
    if(speed < 0 && !LLimit.get()){
      speed = 0;
    }
    if(speed > 0 && !RLimit.get()){
      speed = 0;
    }
    //GantryMotor.set(speed);

  }

 public double Get_enc(){

//return GantryMotor.getSelectedSensorPosition() * Constants.GANTRY_ENC_CONV_FACTOR;
return 0;}

  public void Set_enc(double ENC_set){
    //TurretMotor.setSelectedSensorPosition(Enc_set);
  
  }

  @Override
  public void periodic() {
   // if(TurretMotor.getSelectedSensorVelocity() < 0 && !LLimit.get()){
     //     setSpeed(0);
       //}
      //if(TurretMotor.getSelectedSensorVelocity() > 0 && !RLimit.get()){
        // setSpeed(0);
       //}
  
       //if(LLimit.get() == false){
         //Set_enc(Constants.LIFT_ENC_RESET_HEIGHT);
      // }
      //SmartDashboard.putNumber("Turret Encoder Position", TurretMotor.getSelectedSensorPosition() * Constants.TURRET_ENC_CONV_FACTOR);
    // This method will be called once per scheduler run
  }
}
