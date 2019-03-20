/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Jetson;

public class DriveToDistanceVision extends Command {

  private DriveTrain driveTrain;

  private Jetson jetson;
  
  private final double CURRENT_THRESHOLD = 25;

  private double setpointInches;

  public DriveToDistanceVision(DriveTrain driveTrain, Jetson jetson) {
    this.driveTrain = driveTrain;
    this.jetson = jetson;
    requires(this.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.zeroSensors();
   
    this.setpointInches = jetson.getDirectDistance();
    //this.setpointInches += 7; TODO: adjust this
       
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   driveTrain.setMotionMagicPosition(setpointInches);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return driveTrain.isMotionMagicDone() || driveTrain.getLeftMasterCurrent() > CURRENT_THRESHOLD;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
  
}
