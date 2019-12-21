/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Jetson;

public class DriveTrack extends Command {

  private DriveTrain driveTrain;

  private Jetson jetson;
  private double targetAngle = 0;


  public DriveTrack(DriveTrain driveTrain, Jetson jetson) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.driveTrain = driveTrain;
    this.jetson = jetson;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   targetAngle = jetson.getTargetAngle();
   driveTrain.enableTurnController(targetAngle);
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveTrain.turnToAngle();  
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (driveTrain.getAngle() == targetAngle) {
      return true;
    }



    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
