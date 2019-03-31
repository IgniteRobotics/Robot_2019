/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbOpenLoop extends Command {

  private Climber climber;

  private Joystick manipulatorJoystick;
  private final int THROTTLE_AXIS;
  private final double DEADBAND;

  public ClimbOpenLoop(Climber climber, Joystick manipulatorJoystick, int throttleId, double deadband) {

    this.climber = climber;

    this.THROTTLE_AXIS = throttleId;
    this.DEADBAND = deadband;

    this.manipulatorJoystick = manipulatorJoystick;

    requires(this.climber);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double throttle = manipulatorJoystick.getRawAxis(THROTTLE_AXIS);
    climber.setOpenLoop(throttle, DEADBAND);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
