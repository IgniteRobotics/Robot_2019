/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistanceTimed extends Command {

  private DriveTrain driveTrain;

  private double power;

  public DriveToDistanceTimed(DriveTrain driveTrain, double timeout, double power) {

    this.driveTrain = driveTrain;

    this.power = power;

    requires(this.driveTrain);

    setTimeout(timeout);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveTrain.setOpenLoopLeft(power);
    driveTrain.setOpenLoopRight(power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
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
