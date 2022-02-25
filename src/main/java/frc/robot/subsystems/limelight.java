// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class limelight extends SubsystemBase {
  /** Creates a new limelight. */


  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  double yposition; 
  double xposition = 0; 
  double distancetorpms = 0;
  
public double getTx() {

  return table.getEntry("tx").getDouble(0);
}
public double getTy() {

  return table.getEntry("ty").getDouble(0);
}
  
public double getTa() {

  return table.getEntry("ta").getDouble(0);
}

public double getTs() {

  return table.getEntry("ts").getDouble(0);
}


public double getDistance() {
  setPipeline(0);
  yposition = getTy();
  return Math.pow(yposition,2)*Constants.DIST_CALC_A+Constants.DIST_CALC_B*yposition+Constants.DIST_CALC_C;
}
public double getDistancetoRPMs(double distance){

  setPipeline(0);
  return Math.pow(distance, 2)*Constants.RPMDIST_CALC_A+Constants.RPMDIST_CALC_B*distance+Constants.RPMDIST_CALC_C;
}



  public limelight() {}


public void setPipeline(int pipeline) {
  NetworkTableEntry pipelineTableEntry = table.getEntry("pipeline");
  pipelineTableEntry.setNumber(pipeline); 
  NetworkTableEntry streamTableEntry = table.getEntry("stream");
  streamTableEntry.setNumber(2); 
}

public void setLEDmode(int LEDmode) {
  NetworkTableEntry LEDmodeEntry = table.getEntry("LEDmode");
  LEDmodeEntry.setNumber(LEDmode); 
}




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("targetX", getTx());
    SmartDashboard.putNumber("distance", getDistance());


  }
}
