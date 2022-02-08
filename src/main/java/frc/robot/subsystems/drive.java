/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.setMotors;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Paths;

public class drive extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  // Creates the Master Motors for both sides
  public final WPI_TalonFX FLmotor = new WPI_TalonFX(1);
  public final WPI_TalonFX FRmotor = new WPI_TalonFX(2);


  // Creates the slave(follow) motors for both sides
  private final WPI_TalonFX BLmotor = new WPI_TalonFX(3);
  private final WPI_TalonFX BRmotor = new WPI_TalonFX(4);

  //Creates the gyro object 
  AHRS gyro = new AHRS(SerialPort.Port.kUSB);

  boolean ControlsInverted = false;

  // Created the kinematics class
  //  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics()
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  // Creates variable to store position
  Pose2d pose;
  double test = 0;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);
  
  PIDController leftPIDController = new PIDController(Constants.kp, 0, 0);
  PIDController rightPIDController = new PIDController(Constants.kp, 0, 0);

  public boolean controlreverse = false; 


  public drive() {
    BLmotor.follow(FLmotor);
    BRmotor.follow(FRmotor);

    // sets the polarity for the motors
   FLmotor.setInverted(true);
    BLmotor.setInverted(true);
    FRmotor.setInverted(false);

  }

  public void drivecoast(){
    FRmotor.setNeutralMode(NeutralMode.Coast);
  FLmotor.setNeutralMode(NeutralMode.Coast);
  BRmotor.setNeutralMode(NeutralMode.Coast);
  BLmotor.setNeutralMode(NeutralMode.Coast);
  }

public void drivebrake(){
  FRmotor.setNeutralMode(NeutralMode.Brake);
  FLmotor.setNeutralMode(NeutralMode.Brake);
  BRmotor.setNeutralMode(NeutralMode.Brake);
  BLmotor.setNeutralMode(NeutralMode.Brake);
}




 
  // Creates the Robot's Drivetrain


  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("output", trajectoryName + ".wpilib.json")));
  }
  
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
     FLmotor.getSelectedSensorPosition() * Constants.gearRatio / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches) * Constants.leftScaleConstant,
      FRmotor.getSelectedSensorPosition() * Constants.gearRatio / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches) * Constants.rightScaleConstant
    ); 
  }

  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose){
    resetEncoderPos();
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      this::getPose,
      new RamseteController(2.0, 0.7), 
      feedforward, 
      kinematics, 
      this::getSpeeds, 
      new PIDController(Constants.kp, 0, 0), 
      new PIDController(Constants.kp, 0, 0), 
      this::setOutput, 
      this);
      if (initPose) {
        var reset =  new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
        return reset.andThen(ramseteCommand.andThen(() -> setOutput(0, 0)));
      }
      else {
        return ramseteCommand.andThen(() -> setOutput(0, 0));
      }
  }

  public void setSpeed(double RSpeed, double LSpeed){
    FLmotor.set(LSpeed);
    FRmotor.set(RSpeed);

  }
  
  public Trajectory loadTrajectoryFromFile(String filename) {
    try {
      return loadTrajectory(filename);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return new Trajectory();
    }
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public void setHeading(Pose2d poseMeters, Rotation2d gyroAngle){
    odometry.resetPosition(poseMeters, gyroAngle);
  }

  public double getYaw(){
    return -gyro.getYaw();
  }

  public void resetEncoderPos(){
   FLmotor.setSelectedSensorPosition(0);
    FRmotor.setSelectedSensorPosition(0);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  public void resetEncoders(){
   FLmotor.setSelectedSensorPosition(0);
    FRmotor.setSelectedSensorPosition(0);
  }

  public void setOutput(double leftVolts, double rightVolts){
   FLmotor.set(leftVolts / 12);
    FRmotor.set(rightVolts / 12);
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  @Override
  public void periodic() {
    pose = odometry.update(getHeading(),  FLmotor.getSelectedSensorPosition() * Constants.gearRatio / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches) * Constants.leftScaleConstant,
    FRmotor.getSelectedSensorPosition() * Constants.gearRatio / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches) * Constants.rightScaleConstant);
    test = 0;
    var translation = odometry.getPoseMeters().getTranslation();

    SmartDashboard.putNumber("Left Encoder", FLmotor.getSelectedSensorPosition() * Constants.gearRatio / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches) * Constants.leftScaleConstant);
    SmartDashboard.putNumber("Right Encoder", FRmotor.getSelectedSensorPosition() * Constants.gearRatio / Constants.encoderTicksPerRev * Units.inchesToMeters(Constants.wheelCircumferenceInches) * Constants.rightScaleConstant);
    SmartDashboard.putNumber("Rotation", getYaw());
    SmartDashboard.putNumber("Test", test);
    SmartDashboard.putNumber("wheelCirc", Units.inchesToMeters(Constants.wheelCircumferenceInches));
    SmartDashboard.putNumber("Fused Heading", Constants.encoderTicksPerRev);
    //This stuff is intersting 
  }
}