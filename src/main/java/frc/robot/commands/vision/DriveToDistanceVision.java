/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistanceVision extends Command {

  private DriveTrain driveTrain;

  private boolean lockVisionValues;
  private VisionData visionData;
  private int setpoint;
  
  public DriveToDistanceVision(DriveTrain driveTrain, boolean lockVisionValues, VisionData visionData) {

    this.driveTrain = driveTrain;
    this.lockVisionValues = lockVisionValues;
    this.visionData = visionData;
    
    requires(this.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.zeroSensors();
    if (lockVisionValues) {
      Robot.lockVision();
    }
    if (visionData == VisionData.DISTANCE_1) {
      setpoint = (int)Robot.drive1;
    } else if (visionData == VisionData.DISTANCE_2) {
      setpoint = (int)Robot.drive2;
    }

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   driveTrain.setMotionMagicPosition(setpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return driveTrain.isMotionMagicDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveTrain.stop();
    if (!lockVisionValues) {
      Robot.unlockVision();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.unlockVision();
    end();
  }
  
}
