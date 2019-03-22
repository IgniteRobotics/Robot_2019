/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends Command {

  private DriveTrain driveTrain;
  
  private double setpoint;

  public TurnToAngle(DriveTrain driveTrain, double setpoint) {

    this.driveTrain = driveTrain;
    this.setpoint = setpoint;

    requires(driveTrain);
  
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.zeroAngle();
    driveTrain.enableTurnController(setpoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveTrain.turnToAngle();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return driveTrain.isTurnCompleted();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveTrain.stopTurnController();
    driveTrain.stop();
    driveTrain.zeroAngle();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
  
}
