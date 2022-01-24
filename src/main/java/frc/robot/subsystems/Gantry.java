// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gantry extends SubsystemBase {



  private WPI_TalonFX GantryMotor = new WPI_TalonFX(8);
  /** Creates a new Gantry. */
  private DigitalInput FWDLimit = new DigitalInput(2);
  private DigitalInput BWDLimit = new DigitalInput(3);
  public Gantry() {

    SmartDashboard.putNumber("Test Height", 0);

  }

  public void gantrysetSpeed(double speed) {
    if(speed < 0 && !BWDLimit.get()){
      speed = 0;
    }
    if(speed > 0 && !FWDLimit.get()){
      speed = 0;
    }
    GantryMotor.set(speed);

  }

 public double Get_enc(){

return GantryMotor.getSelectedSensorPosition() * Constants.GANTRY_ENC_CONV_FACTOR;

 }
  public void Set_enc(double Enc_set){
    GantryMotor.setSelectedSensorPosition(Enc_set);
  }

  @Override
  public void periodic() {


    if(GantryMotor.getSelectedSensorVelocity() < 0 && !BWDLimit.get()){
       gantrysetSpeed(0);
    }
    if(GantryMotor.getSelectedSensorVelocity() > 0 && !FWDLimit.get()){
      gantrysetSpeed(0);
    }

    if(BWDLimit.get() == false){
      Set_enc(Constants.LIFT_ENC_RESET_HEIGHT);
    }
    SmartDashboard.putNumber("Gantry Encoder Position", GantryMotor.getSelectedSensorPosition() * Constants.GANTRY_ENC_CONV_FACTOR);
    // This method will be called once per scheduler run
  }
}
