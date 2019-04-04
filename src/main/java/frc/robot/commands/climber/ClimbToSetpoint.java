/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbToSetpoint extends Command {

  private Climber climber;

  private int setpoint;

  public ClimbToSetpoint(Climber climber, int setpoint) {

    this.climber = climber;
    this.setpoint = setpoint;

    requires(this.climber);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    climber.setMotionMagicPosition(setpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (setpoint == 0) {
      return climber.isMotionMagicDone() || climber.isFwdLimitTripped(); // TODO: DETERMINE IF FWD IS BOTTON
    } else {
      return climber.isMotionMagicDone();
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (climber.isFwdLimitTripped()) {
      climber.zeroSensors();
    }
    climber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
  
}
