/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends Command {

  private DriveTrain driveTrain;
  
  private int setpoint_inches;
  private final double CURRENT_THRESHOLD = 15;
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Vision");

  public DriveToDistance(DriveTrain driveTrain, int setpoint_inches) {

    this.driveTrain = driveTrain;
    this.setpoint_inches = setpoint_inches;
    
    requires(this.driveTrain);
  }

  public DriveToDistance(DriveTrain driveTrain)
  {
    this.driveTrain = driveTrain;
    
    requires(this.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.zeroSensors();
   
    if (this.setpoint_inches == 0)
    {
      this.setpoint_inches = table.getEntry("DIRECT_DISTANCE").getNumber(0).intValue();
      this.setpoint_inches += 7;
    }
       
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

   // driveTrain.setOpenLoopLeft(.3);
   //  driveTrain.setOpenLoopRight(.3);
   driveTrain.setMotionMagicPosition(setpoint_inches);
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
