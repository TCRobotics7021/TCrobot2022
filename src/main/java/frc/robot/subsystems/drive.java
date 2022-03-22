/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Paths;

public class drive extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  // Motors defined
  public final WPI_TalonFX FLmotor = new WPI_TalonFX(1);
  public final WPI_TalonFX FRmotor = new WPI_TalonFX(2);
  private final WPI_TalonFX BLmotor = new WPI_TalonFX(3);
  private final WPI_TalonFX BRmotor = new WPI_TalonFX(4);

  //Gyro defined 
  AHRS gyro = new AHRS(SerialPort.Port.kUSB);

  // kinematics and odometry defined
  // the odometry object keeps track of where the robot is on the field
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.track_width);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  //Defines a variable to store position
  //Pose2d pose;

  //Defines a feedforward system using the constants found from the characterization tool
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);
  
  //Defines the PID controllers for the left and right drives using kp found from the characterization tool
  PIDController leftPIDController = new PIDController(Constants.kp, 0, 0);
  PIDController rightPIDController = new PIDController(Constants.kp, 0, 0);

  //Controls are inverted if this is true
  public boolean controlreverse = false; 


  public drive() {
    // sets the follower motors to follow
    BLmotor.follow(FLmotor);
    BRmotor.follow(FRmotor);

    // sets the polarity for the motors
    FLmotor.setInverted(true);
    BLmotor.setInverted(true);
    FRmotor.setInverted(false);
    BRmotor.setInverted(false);

  }

  //When this mode is active the motors will coast when given a speed of 0;
  public void drivecoast(){
    FRmotor.setNeutralMode(NeutralMode.Coast);
    FLmotor.setNeutralMode(NeutralMode.Coast);
    BRmotor.setNeutralMode(NeutralMode.Coast);
    BLmotor.setNeutralMode(NeutralMode.Coast);
  }

  //When this mode is active the motors will brake when given a speed of 0;
  public void drivebrake(){
    FRmotor.setNeutralMode(NeutralMode.Brake);
    FLmotor.setNeutralMode(NeutralMode.Brake);
    BRmotor.setNeutralMode(NeutralMode.Brake);
    BLmotor.setNeutralMode(NeutralMode.Brake);
  }

 

  //Returns the wheel speeds (needed for ramsete function)
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    FLmotor.getSelectedSensorVelocity() * Constants.metersPerEncoderTick / 10,
    FRmotor.getSelectedSensorVelocity() * Constants.metersPerEncoderTick / 10);
  }

  //These two functions load the path file and generate a trajectory
  public Trajectory loadTrajectoryFromFile(String filename) {
    try {
      return loadTrajectory(filename);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return new Trajectory();
    }
  }
  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    //return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths/", trajectoryName + ".wpilib.json")));
    return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("pathplanner/generatedJSON/", trajectoryName + ".wpilib.json")));
    //return TrajectoryUtil.fromPathweaverJson(path)
  }

  //This is the ramsete function, it creates an autonomous command from a trajectory
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

  //Sets the motor speeds based on percent output
  public void setSpeed(double RSpeed, double LSpeed){
    FLmotor.set(LSpeed);
    FRmotor.set(RSpeed);
  }

  //Sets the motor speeds based on voltage
  public void setOutput(double leftVolts, double rightVolts){
     FLmotor.setVoltage(leftVolts);
     FRmotor.setVoltage(rightVolts);
   }
  
  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }
 //Returns the current heading
  public Rotation2d getHeading() {
    //return Rotation2d.fromDegrees(-gyro.getYaw());
    return gyro.getRotation2d();
  }
  //tells the odometry object where the robot currently is
  public void setHeading(Pose2d poseMeters, Rotation2d gyroAngle){
    odometry.resetPosition(poseMeters, gyroAngle);
  }

  //resets the value of the encoders to 0
  public void resetEncoderPos(){
    FLmotor.setSelectedSensorPosition(0);
    FRmotor.setSelectedSensorPosition(0);
  }

  //Returns the current position from the odometry object
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public PIDController getLeftPIDController(){
    return leftPIDController;
  }

  public PIDController getRightPIDController(){
    return rightPIDController;
  }

  public void resetOdometry(Pose2d pose){
    resetEncoderPos();
    odometry.resetPosition(pose, getHeading());
  }

  @Override
  public void periodic() {
    
    //This updates the odometry object so it can keep track of where the robot is
    odometry.update(getHeading(),  FLmotor.getSelectedSensorPosition() * Constants.metersPerEncoderTick, FRmotor.getSelectedSensorPosition() * Constants.metersPerEncoderTick);
    //pose = odometry.update(getHeading(),  FLmotor.getSelectedSensorPosition() * Constants.metersPerEncoderTick, FRmotor.getSelectedSensorPosition() * Constants.metersPerEncoderTick);
    var translation = odometry.getPoseMeters().getTranslation();
    if(Constants.SHOW_DATA){
      SmartDashboard.putNumber("Odometry X", translation.getX());
      SmartDashboard.putNumber("Odometry Y", translation.getY());
      SmartDashboard.putNumber("Left Encoder", FLmotor.getSelectedSensorPosition() * Constants.metersPerEncoderTick);
      SmartDashboard.putNumber("Right Encoder", FRmotor.getSelectedSensorPosition() * Constants.metersPerEncoderTick);
      SmartDashboard.putNumber("Odometry Rotation Degrees", odometry.getPoseMeters().getRotation().getDegrees());
      SmartDashboard.putNumber("Odometry Rotation Radians", odometry.getPoseMeters().getRotation().getRadians());
      SmartDashboard.putNumber("Left Velocity m/s", FLmotor.getSelectedSensorVelocity() * Constants.metersPerEncoderTick / 10);
      SmartDashboard.putNumber("Right Velocity m/s", FRmotor.getSelectedSensorVelocity() * Constants.metersPerEncoderTick / 10);
    }
    //This stuff is intersting 
  }
}