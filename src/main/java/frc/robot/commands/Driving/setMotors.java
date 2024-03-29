
package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class setMotors extends CommandBase {
  /** Creates a new printVal. */
  double leftSpeed;
  double rightSpeed;
  boolean finished;

  public setMotors(double leftSpeed, double rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive_subsystem);
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    finished = false;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Done");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive_subsystem.setOutput(leftSpeed, rightSpeed);
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}