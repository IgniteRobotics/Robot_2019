/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistanceTimedConditional extends Command {

  private DriveTrain driveTrain;
  private Carriage carriage;

  private double power;

  public DriveToDistanceTimedConditional(DriveTrain driveTrain, Carriage carriage, double timeout, double power) {
    this.driveTrain = driveTrain;
    this.carriage = carriage;

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
    if (!carriage.hasCargo()) {
      driveTrain.setOpenLoopLeft(power);
      driveTrain.setOpenLoopRight(power);
    } else {
      end();
    }
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
